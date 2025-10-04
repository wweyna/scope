/*
 * scope.c
 *
 *  Created on: Nov 11, 2024
 *      Author: Wojciech Weyna
 */

#include "main.h"

#include "scope.h"
#include "vdda.h"

#include <math.h>
#include <string.h>
#include <stdbool.h>

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

// ----------- External handles to peripherals  -----------

extern ADC_HandleTypeDef    hadc1;
extern ADC_HandleTypeDef    hadc3;
extern DMA_HandleTypeDef    hdma_adc1;
extern UART_HandleTypeDef   hlpuart1;
extern DMA_HandleTypeDef    hdma_lpuart1_tx;

// ----------- Scope constant parameters -----------

/// All ADC parameters are unit16_t samples (2 bytes)
#define SCOPE_ADC_SAMPLE_SIZE                   (2)

/// Maximum available ADC data frame size in unit16_t samples
#define SCOPE_ADC_MAX_FRAME_SIZE                (1154)

/// Half of ADC buffer can hold maximum of SCOPE_ADC_FRAMES_IN_HALF_BUFFER frames of maximum size
/// Increasing this number gives more time for processing in ADC interrupt.
#define SCOPE_ADC_FRAMES_IN_HALF_BUFFER         (2)

/// Half of ADC DMA buffer size in unit16_t samples
#define SCOPE_ADC_MAX_DMA_BUFFER_HALF_SIZE      (SCOPE_ADC_MAX_FRAME_SIZE * SCOPE_ADC_FRAMES_IN_HALF_BUFFER)

/// ADC DMA buffer size in unit16_t samples
#define SCOPE_ADC_MAX_DMA_BUFFER_SIZE           (SCOPE_ADC_MAX_DMA_BUFFER_HALF_SIZE * 2)

// All OUT parameters are unit8_t (1 byte)

// Frame header:
//      0xFF 0xFE - header marker
//      0x00 0x00 - VREF
//      0x00      - time base code

#define SCOPE_OUT_FRAME_HEADER_SIZE (5)                         // OUTPUT frame header length in bytes

#define SCOPE_OUT_MAX_FRAME_DATA_SIZE   (SCOPE_ADC_MAX_FRAME_SIZE * SCOPE_ADC_SAMPLE_SIZE)  // Maximum allowed OUTPUT frame data size in bytes
#define SCOPE_OUT_MAX_FRAME_SIZE        (SCOPE_OUT_MAX_FRAME_DATA_SIZE + SCOPE_OUT_FRAME_HEADER_SIZE)  // Maximu allowed OUTPUT frame size in bytes (with header)
#define SCOPE_OUT_MAX_DMA_BUFFER_SIZE    SCOPE_OUT_MAX_FRAME_SIZE  // We send single frame at a time

#define SCOPE_DEFAULT_TIME_BASE         kTimeBase_10ms

/// Maximum value of ADC conversion for 12 bit adc resolution
#define SCOPE_ADC_MAX_VALUE __LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) // 4095


#define SCOPE_MARGIN_FRAME_COUNT         1
#define SCOPE_RECORD_BUFFER_FRAME_COUNT  SCOPE_ADC_FRAMES_IN_HALF_BUFFER * 2
#define SCOPE_RECORD_BUFFER_MAX_SIZE     ((SCOPE_RECORD_BUFFER_FRAME_COUNT + SCOPE_MARGIN_FRAME_COUNT * 2) * SCOPE_ADC_MAX_FRAME_SIZE)
#define SCOPE_RECORD_ACQUSITION_COUNT    (SCOPE_RECORD_BUFFER_FRAME_COUNT / SCOPE_ADC_FRAMES_IN_HALF_BUFFER)

// ----------- Types -----------------

/// Scope run states
enum RunState
{
    ///
    kRunState_Invalid = 0,
    ///
    kRunState_Stopped = 1,
    ///
    kRunState_Running = 2,
};

/// Time base descriptor defines frame acquisition parameters like sampling rate and frame size.
struct TimeBaseDescriptor
{
    enum TimeBaseCode code;
    uint16_t frameSize;
    uint32_t samplingTime;
};

/// Time base descriptors table contains all fixed time bases supported by scope
struct TimeBaseDescriptor scope_time_base_descriptors[] = {
    {kTimeBase_Invalid, 0,      0},
    {kTimeBase_10ms,    1154,   ADC_SAMPLETIME_247CYCLES_5},
    {kTimeBase_1ms,     811,    ADC_SAMPLETIME_24CYCLES_5},
    {kTimeBase_250ms,   500,    ADC_SAMPLETIME_2CYCLES_5},
    {kTimeBase_End,     0xFFFF, 0}
};


///
enum RecordingStatus
{
    /// ADC sampler is not working
    kRecordingStatus_Idle,
    /// Recording in progress, record buffer is locked
    kRecordingStatus_InProgress,
    /// Recording completed, record buffer is unlocked
    kRecordingStatus_Completed,
};

///
enum ADCBufferStatus
{
    /// Data acquisition in the left buffer is completed
    kADCBufferStatus_LeftHalf,
    /// Data acquisition in the right buffer is completed
    kADCBufferStatus_RightHalf
};

/// Output frame transmission status
enum OutDataTxStatus
{
    /// There is no transmission in progress (transmission hasn't started yet).
    kOutDataTxStatus_Idle,
    /// Frame transmission has finished.
    kOutDataTxStatus_Completed,
    /// Frame transmission is in progress
    kOutDataTxStatus_InProgress
};


enum ScopeCommand
{
    kCmd_None,
    kCmd_Run,
    kCmd_Stop,
    kCmd_SetTimeBase
};

// ----------- Local data  -----------
uint16_t scope_adc_vdda_voltage;

const uint16_t scope_trigger_position_denominator = 10000;
const uint16_t scope_trigger_position_50percent = scope_trigger_position_denominator / 2;
uint16_t scope_trigger_position = scope_trigger_position_50percent;

enum ScopeCommand scope_requested_command = kCmd_None;
enum TimeBaseCode scope_requested_new_time_base_code = kTimeBase_Invalid;
enum TimeBaseCode scope_time_base_code = SCOPE_DEFAULT_TIME_BASE;
enum RecordingStatus scope_recording_status = kRecordingStatus_Idle;
enum RunState scope_run_state = kRunState_Invalid;
enum ScopeEdgeType scope_edge_type = kScopeEdge_Rising;
enum ScopeEdgeType scope_requested_edge_type = kScopeEdge_Rising;
enum ScopeTriggerType scope_trigger_type = kScopeTrigger_Auto;
enum ScopeTriggerType scope_requested_trigger_type = kScopeTrigger_Auto;
enum OutDataTxStatus scope_out_data_tx_status = kOutDataTxStatus_Idle;


// ADC DMA ring-buffer.
// This buffer records a continues stream of ADC samples in real-time.

uint16_t scope_adc_dma_buffer[SCOPE_ADC_MAX_DMA_BUFFER_SIZE];

uint16_t scope_adc_frame_size;
uint16_t scope_adc_frame_size_bytes;
uint16_t scope_adc_dma_buffer_half_size;
uint16_t scope_adc_dma_buffer_size;
uint16_t* scope_adc_dma_buffer_left_begin;
uint16_t* scope_adc_dma_buffer_right_begin;

// Record buffer

uint16_t scope_record_buffer[SCOPE_RECORD_BUFFER_MAX_SIZE];
uint16_t scope_record_counter;
uint16_t scope_record_buffer_size;

// Output frame double-buffer.

uint8_t scope_out_frame_buffer_1[SCOPE_OUT_MAX_DMA_BUFFER_SIZE];
uint8_t scope_out_frame_buffer_2[SCOPE_OUT_MAX_DMA_BUFFER_SIZE];

uint8_t scope_out_encoded_frame_buffer[SCOPE_OUT_MAX_DMA_BUFFER_SIZE];
uint16_t scope_out_encoded_frame_buffer_size = 0;

uint8_t* scope_out_frame_front_buffer = scope_out_frame_buffer_1;
uint8_t* scope_out_frame_back_buffer = scope_out_frame_buffer_2;

uint16_t scope_out_frame_data_size;     // frame size without header
uint16_t scope_out_frame_buffer_size;   // frame full size


// ------------- local scope functions ---------------

static
bool scope_is_time_base_code_valid(enum TimeBaseCode timeBaseCode)
{
    return timeBaseCode > kTimeBase_Invalid && timeBaseCode < kTimeBase_End;
}

static
void scope_reset_status()
{
    scope_recording_status = kRecordingStatus_Idle;
    scope_record_counter = 0;
    scope_requested_command = kCmd_None;
}

/**
 * This method sets all buffer sizes and pointers.
 * Before it is called, scope adc and transmission must be stopped.
 */
static
void scope_select_frame_size(int adc_frame_size)
{
    scope_adc_frame_size = adc_frame_size;
    scope_adc_frame_size_bytes = scope_adc_frame_size * SCOPE_ADC_SAMPLE_SIZE;

    scope_adc_dma_buffer_half_size = scope_adc_frame_size * SCOPE_ADC_FRAMES_IN_HALF_BUFFER;
    scope_adc_dma_buffer_size = scope_adc_dma_buffer_half_size * 2;

    scope_adc_dma_buffer_left_begin = scope_adc_dma_buffer;
    scope_adc_dma_buffer_right_begin = scope_adc_dma_buffer_left_begin + scope_adc_dma_buffer_half_size;

    scope_out_frame_data_size = scope_adc_frame_size_bytes;
    scope_out_frame_buffer_size = scope_out_frame_data_size + SCOPE_OUT_FRAME_HEADER_SIZE;

    scope_record_buffer_size = SCOPE_RECORD_BUFFER_FRAME_COUNT * scope_adc_frame_size;
}

static
void scope_setup_frame_header()
{
    const uint8_t scope_adc_vdda_voltage_lsb = scope_adc_vdda_voltage & 0x00FF;
    const uint8_t scope_adc_vdda_voltage_msb = scope_adc_vdda_voltage >> 8;

    scope_out_frame_buffer_1[0] = 0xFF;
    scope_out_frame_buffer_1[1] = 0xFE;
    scope_out_frame_buffer_1[2] = scope_adc_vdda_voltage_lsb;
    scope_out_frame_buffer_1[3] = scope_adc_vdda_voltage_msb;
    scope_out_frame_buffer_1[4] = scope_time_base_code;

    scope_out_frame_buffer_2[0] = 0xFF;
    scope_out_frame_buffer_2[1] = 0xFE;
    scope_out_frame_buffer_2[2] = scope_adc_vdda_voltage_lsb;
    scope_out_frame_buffer_2[3] = scope_adc_vdda_voltage_msb;
    scope_out_frame_buffer_2[4] = scope_time_base_code;
}

static
void scope_reset_buffers()
{
    assert(SCOPE_ADC_FRAMES_IN_HALF_BUFFER >= 2);
    assert(SCOPE_ADC_FRAMES_IN_HALF_BUFFER % 2 == 0);
    assert(SCOPE_RECORD_BUFFER_FRAME_COUNT >= 2);
    assert(SCOPE_RECORD_BUFFER_FRAME_COUNT % SCOPE_ADC_FRAMES_IN_HALF_BUFFER == 0);

    memset((char*)scope_adc_dma_buffer, 0, SCOPE_ADC_MAX_DMA_BUFFER_SIZE * SCOPE_ADC_SAMPLE_SIZE);
    memset((char*)scope_record_buffer, 0, SCOPE_RECORD_BUFFER_MAX_SIZE * SCOPE_ADC_SAMPLE_SIZE);
    memset((char*)scope_out_frame_buffer_1, 0, SCOPE_OUT_MAX_DMA_BUFFER_SIZE);
    memset((char*)scope_out_frame_buffer_2, 0, SCOPE_OUT_MAX_DMA_BUFFER_SIZE);
    scope_out_frame_front_buffer = scope_out_frame_buffer_1;
    scope_out_frame_back_buffer = scope_out_frame_buffer_2;
    scope_setup_frame_header();
}

static
void scope_stop_adc_sampler()
{
    if (scope_run_state == kRunState_Running)
    {
        HAL_ADC_Stop_DMA(&hadc1);
        scope_run_state = kRunState_Stopped;
    }
    else
    {
        assert(false);
    }
}

static
void scope_callibrate()
{
    if (scope_run_state != kRunState_Running)
    {
        scope_adc_vdda_voltage = VDDA_GetVoltage();

        HAL_ADCEx_Calibration_Start (&hadc1, ADC_SINGLE_ENDED);

        // Safe delay between ADC end of calibration and ADC enable.
        int wait_loop_index = LL_ADC_DELAY_CALIB_ENABLE_ADC_CYCLES * 100;
        while(wait_loop_index != 0)
        {
          wait_loop_index--;
        }
    }
    else
    {
        // Cannot callibrate while running
        // (this function assumes that caller calls it in correct context)
        assert(false);
    }
}

static
void scope_start_adc_sampler(){

    if (scope_run_state != kRunState_Running)
    {
        if (scope_is_time_base_code_valid(scope_time_base_code) == false)
        {
            assert(false);
            return;
        }

        scope_reset_status();
        scope_reset_buffers();

        scope_select_frame_size(scope_time_base_descriptors[scope_time_base_code].frameSize);

        ADC_ChannelConfTypeDef sConfig = {0};
        sConfig.Channel = ADC_CHANNEL_1;
        sConfig.Rank = ADC_REGULAR_RANK_1;
        sConfig.SamplingTime = scope_time_base_descriptors[scope_time_base_code].samplingTime;
        sConfig.SingleDiff = ADC_SINGLE_ENDED;
        sConfig.OffsetNumber = ADC_OFFSET_NONE;
        sConfig.Offset = 0;
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        {
            Error_Handler();
        }

        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)scope_adc_dma_buffer, scope_adc_dma_buffer_size);

        scope_run_state = kRunState_Running;
    }
    else
    {
        // Sampler already running
        // (this function assumes that caller calls it in correct context)
        assert(false);
    }
}

static
void scope_select_time_base(enum TimeBaseCode timeBaseCode)
{
    if (scope_time_base_code != timeBaseCode)
    {
        if (scope_is_time_base_code_valid(timeBaseCode) == false)
        {
            assert(false);
            return;
        }
        scope_time_base_code = timeBaseCode;
        if (scope_run_state == kRunState_Running)
        {
            scope_stop_adc_sampler();
            scope_start_adc_sampler();
        }
    }
}

static
void scope_swap_out_buffers()
{
    // Cannot swap output buffer if transmission is in progress.
    // Make sure that transmission is finished first
    if (scope_out_data_tx_status == kOutDataTxStatus_InProgress)
    {
        assert(false);
        return;
    }

    // Swap buffers.
    uint8_t* tmp = scope_out_frame_front_buffer;
    scope_out_frame_front_buffer = scope_out_frame_back_buffer;
    scope_out_frame_back_buffer = tmp;

    assert (scope_out_frame_front_buffer != scope_out_frame_back_buffer);
}

static
void scope_copy_frame_to_backbuffer(const uint16_t* adc_frame_begin)
{
    uint8_t* const out_buffer_begin = scope_out_frame_back_buffer + SCOPE_OUT_FRAME_HEADER_SIZE;
    memcpy(out_buffer_begin, adc_frame_begin, scope_out_frame_data_size);
}


static
uint16_t scope_get_trigger_offset()
{
    const double offset_factor = scope_trigger_position / (double)scope_trigger_position_denominator;
    return scope_adc_frame_size * offset_factor;
}

static
uint16_t* scope_find_trigger()
{
    uint16_t* adc_trigger_pointer = 0;

    uint16_t* data_begin = scope_record_buffer + scope_adc_frame_size * SCOPE_MARGIN_FRAME_COUNT;
    uint16_t* begin = data_begin + scope_get_trigger_offset();
    uint16_t* end = data_begin + scope_record_buffer_size - scope_get_trigger_offset();

    // Auto find trigger - use mean signal value as trigger value

    uint16_t min_value = -1;
    uint16_t max_value = 0;

    for (uint16_t* it = begin; it < end; ++it)
    {
        const uint16_t value = *it;
        min_value = MIN(min_value, value);
        max_value = MAX(max_value, value);
    }

    const uint16_t half_amplitude = (max_value - min_value) >> 1;
    const uint16_t mean_value = min_value + half_amplitude;
    const uint16_t scope_trigger_value = mean_value;


    // Trigger when value change passes thru scope_trigger_value

    uint16_t start_value = *begin;
    int trigger_detection_counter = 0;
    const int trigger_detection_range = 3;
    for (uint16_t* it = begin; it < end; ++it)
    {
        const uint16_t value = *it;

        bool isTrigger = false;
        switch (scope_edge_type)
        {
            case kScopeEdge_Rising:
                isTrigger = start_value < scope_trigger_value && value >= scope_trigger_value;
                break;
            case kScopeEdge_Falling:
                isTrigger = start_value > scope_trigger_value && value <= scope_trigger_value;
                break;
            default:
                assert(false);
        }

        if (isTrigger)
        {
            if (++trigger_detection_counter >= trigger_detection_range)
            {
                adc_trigger_pointer = it - trigger_detection_counter;
                if (adc_trigger_pointer < begin)
                {
                    adc_trigger_pointer = begin;
                    assert(false);
                }
                break;
            }
        }
        else
        {
            trigger_detection_counter = 0;
            start_value = value;
        }
    }

    return adc_trigger_pointer;
}

static
uint16_t* scope_get_start_of_frame_for_trigger(uint16_t* adc_trigger_pointer)
{
    uint16_t* start_of_frame = adc_trigger_pointer - scope_get_trigger_offset();
    if (start_of_frame < scope_record_buffer)
    {
        assert(false);
        start_of_frame = scope_record_buffer;
    }
    return start_of_frame;
}

static
uint16_t* scope_first_frame()
{
    uint16_t* begin = scope_record_buffer + scope_adc_frame_size * SCOPE_MARGIN_FRAME_COUNT;
    return begin;
}

/**
 * Return a pointer to left or right part of ADC buffer, depending on the ADCBufferStatus.
 */
static
uint16_t* scope_adc_dma_buffer_begin(enum ADCBufferStatus adc_acquisition_status)
{
    switch (adc_acquisition_status)
    {
    case kADCBufferStatus_LeftHalf:
        return scope_adc_dma_buffer_left_begin;
    case kADCBufferStatus_RightHalf:
        return scope_adc_dma_buffer_right_begin;
    default:
        assert(false);
        return scope_adc_dma_buffer_left_begin;
    };
}

void scope_process_requested_commands()
{
    if (scope_requested_command == kCmd_SetTimeBase && scope_requested_new_time_base_code > 0)
    {
        scope_requested_command = kCmd_None;
        scope_select_time_base(scope_requested_new_time_base_code);
        scope_requested_new_time_base_code = 0;
    }
    else if (scope_requested_command == kCmd_Stop)
    {
        scope_requested_command = kCmd_None;
        Scope_Stop();
    }
    else if (scope_requested_command == kCmd_Run)
    {
        scope_requested_command = kCmd_None;
        Scope_Run();
    }
}

static
void scope_on_adc_data_ready(enum ADCBufferStatus adc_acquisition_status)
{
    if (scope_recording_status == kRecordingStatus_Completed)
    {
        // Nothing to do, data already in buffer and reader didn't have a chance to read it yet.
        // Reader will reset scope_recording_status to kRecordingStatus_Idle after it reads the data.
        return;
    }

    if (scope_recording_status == kRecordingStatus_Idle)
    {
        // Start recording
        scope_recording_status = kRecordingStatus_InProgress;
        assert(scope_record_counter == 0);
        scope_record_counter = 0;
    }
    else if (scope_recording_status == kRecordingStatus_InProgress)
    {
        assert(scope_record_counter > 0);
    }
    else
    {
        assert(false);
    }

    // Append next ADC data frame (s) to record buffer

    uint16_t* src = scope_adc_dma_buffer_begin(adc_acquisition_status);
    const uint16_t next_frame_pos = scope_record_counter * scope_adc_frame_size * SCOPE_ADC_FRAMES_IN_HALF_BUFFER;
    uint16_t* dest = scope_record_buffer + scope_adc_frame_size * SCOPE_MARGIN_FRAME_COUNT + next_frame_pos;

    assert(dest + scope_adc_frame_size * SCOPE_MARGIN_FRAME_COUNT <= scope_record_buffer + SCOPE_RECORD_BUFFER_MAX_SIZE);

    memcpy(dest, src, scope_adc_frame_size_bytes * SCOPE_ADC_FRAMES_IN_HALF_BUFFER);

    scope_record_counter++;

    if (scope_record_counter >= SCOPE_RECORD_ACQUSITION_COUNT)
    {
        scope_record_counter = 0;
        scope_recording_status = kRecordingStatus_Completed;
    }
}

/// Right ADC buffer
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        scope_on_adc_data_ready(kADCBufferStatus_RightHalf);
    }
}

/// Left ADC buffer
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        scope_on_adc_data_ready(kADCBufferStatus_LeftHalf);
    }
}

void
HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    scope_out_data_tx_status = kOutDataTxStatus_Completed;
}

void scope_encode_output_frame()
{
    // Set output encoded frame header
    memcpy (scope_out_encoded_frame_buffer, scope_out_frame_front_buffer, SCOPE_OUT_FRAME_HEADER_SIZE);

    // Encode / pack output frame to make it smaller.

    // For 10-bit transport sample.
    // ADC data is reduced to 10 bit sample for transport
    // The pack of 3 10bit samples is encoded into 2 samples by encoding 3rd sample into free 5-bit space of first 2 samples
    // (12,12,12) -> (10,10,10) -> (10 + 5, 10 + 5)
    // The encoded 3 sample pack contains data in first 15 bits.
    // Last byte is reserved for special codes (like frame start).

    const uint16_t src_pack_size_samples = 3;
    const uint16_t dest_pack_size_samples = src_pack_size_samples - 1;
    const uint16_t src_pack_size = src_pack_size_samples * SCOPE_ADC_SAMPLE_SIZE;
    const uint16_t pack_count = scope_out_frame_data_size / src_pack_size;
    const uint16_t tail_bytes = scope_out_frame_data_size % src_pack_size;

    // The data that will be encoded
    const uint16_t src_encoded_data_size = pack_count * src_pack_size_samples;
    assert(src_encoded_data_size * SCOPE_ADC_SAMPLE_SIZE  + tail_bytes == scope_out_frame_data_size);

    // The encoded (packed) data.
    const uint16_t dest_encoded_data_size = pack_count * dest_pack_size_samples;
    scope_out_encoded_frame_buffer_size = SCOPE_OUT_FRAME_HEADER_SIZE + dest_encoded_data_size * SCOPE_ADC_SAMPLE_SIZE + tail_bytes;

    // Source range to be encoded
    uint16_t* const src_begin = (uint16_t*)(scope_out_frame_front_buffer + SCOPE_OUT_FRAME_HEADER_SIZE);
    uint16_t* const src_end = src_begin + src_encoded_data_size;

    // Destination range
    uint16_t* dest = (uint16_t*)(scope_out_encoded_frame_buffer + SCOPE_OUT_FRAME_HEADER_SIZE);
    uint16_t* dest_end = dest + dest_encoded_data_size;

    // Encode
    for (uint16_t* src = src_begin; src < src_end; src++)
    {
        const uint16_t* src_last_sample = (uint16_t*)(src + dest_pack_size_samples);
        const uint16_t last_sample_10bit = (*src_last_sample) >> 2;
        // 5 bits from last sample are added at bit 10-14 0b0xxxxxnnnnnnn
        // bit 15 is reserved for special codes that use bit 15 like start of frame (0xFFFE)
        *dest++ = ((*src++) >> 2) | ((last_sample_10bit & 0b0000000000011111) << 10);
        *dest++ = ((*src++) >> 2) | ((last_sample_10bit & 0b0000001111100000) << 5);
    }

    assert (dest == dest_end);

    // Copy tail data (remaining unencoded samples)
    const uint16_t tail_samples = tail_bytes / SCOPE_ADC_SAMPLE_SIZE;
    for (uint16_t* src = src_end; src < src_end + tail_samples; src++)
    {
        *dest++ = (*src) >> 2;
    }
}

uint16_t*
scope_get_start_of_output_frame()
{
    uint16_t* output_frame = 0;
    if (scope_trigger_type == kScopeTrigger_Auto)
    {
       output_frame = scope_find_trigger();
       if (output_frame)
       {
           output_frame = scope_get_start_of_frame_for_trigger(output_frame);
       }
    }

    if (output_frame == 0)
    {
        // Trigger not found or no trigger mode, just take the frist frame
        output_frame = scope_first_frame();
    }

    return output_frame;
}

//static
//void scope_set_trigger_position_x(uint16_t x_position)
//{
//    scope_trigger_position = x_position;
//    if (scope_trigger_position >= scope_trigger_position_denominator)
//    {
//        scope_trigger_position = scope_trigger_position_denominator - 1;
//    }
//}

// -------------------- Scope external API ----------------

void Scope_Run()
{
    if (scope_run_state != kRunState_Running)
    {
        if (scope_run_state == kRunState_Invalid)
        {
            scope_callibrate();
        }
        scope_start_adc_sampler();
    }
}

void Scope_Stop()
{
    if (scope_run_state != kRunState_Stopped)
    {
        scope_stop_adc_sampler();
    }
}

void Scope_SetEdge(enum ScopeEdgeType edgeType)
{
    scope_requested_edge_type = edgeType;
    if (scope_run_state != kRunState_Running)
    {
        scope_edge_type = edgeType;
    }
}

void Scope_SetTrigger(enum ScopeTriggerType triggerType)
{
    scope_requested_trigger_type = triggerType;
    if (scope_run_state != kRunState_Running)
    {
        scope_trigger_type = triggerType;
    }
}

void Scope_SetTimeBase(enum TimeBaseCode timeBase)
{
    scope_requested_command = kCmd_SetTimeBase;
    scope_requested_new_time_base_code = timeBase;
}


void Scope_TransmitFrame()
{
    // Wait until recording is finished.
    // Process incoming requests while waiting.
    for(;;)
    {
        scope_process_requested_commands();
        if (scope_run_state == kRunState_Stopped)
        {
            break;
        }
        if (scope_recording_status == kRecordingStatus_Completed)
        {
            break;
        }
    }

    // Update acquisition parameters.
    scope_edge_type = scope_requested_edge_type;
    scope_trigger_type = scope_requested_trigger_type;

    // Find trigger position in buffer
    const uint16_t* output_frame_source = scope_get_start_of_output_frame();
    assert (output_frame_source != 0);

    // Copy output frame from record buffer to output back-buffer

    if (output_frame_source != 0)
    {
        scope_copy_frame_to_backbuffer(output_frame_source);
    }

    // Unlock record buffer

    scope_recording_status = kRecordingStatus_Idle;

    // Wait for the UART DMA to finish the frame transmission (wait for HAL_UART_TxCpltCallback())

    uint32_t tickstart = HAL_GetTick();
    const uint32_t txTimeout = 1000;
    while (scope_out_data_tx_status != kOutDataTxStatus_Completed && (HAL_GetTick() - tickstart) < txTimeout);
    if (scope_out_data_tx_status != kOutDataTxStatus_Completed)
    {
        // Timeout occurred.
        // Force status kOutDataTxStatus_Completed in case of timeout.
        scope_out_data_tx_status = kOutDataTxStatus_Completed;
    }

    // Swap output buffers

    scope_swap_out_buffers();

    // Encode frame and start transmission

    scope_out_data_tx_status = kOutDataTxStatus_InProgress;

    // Pack (scope_out_frame_front_buffer, scope_out_frame_buffer_size)
    // to (scope_out_encoded_frame_buffer, scope_out_encoded_frame_buffer_size)
    scope_encode_output_frame();

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&hlpuart1, scope_out_encoded_frame_buffer, scope_out_encoded_frame_buffer_size);

    UNUSED(status);

    // Ensure a small idle time between frames.
    HAL_Delay (1);
}

void Scope_OnCommandReceived(uint8_t cmd)
{
    if (cmd == 'R')
    {
        Scope_SetEdge(kScopeEdge_Rising);
    }
    else if (cmd == 'F')
    {
        Scope_SetEdge(kScopeEdge_Falling);
    }
    else if (cmd == 'A')
    {
        Scope_SetTrigger(kScopeTrigger_Auto);
    }
    else if (cmd == 'N')
    {
        Scope_SetTrigger(kScopeTrigger_None);
    }
    else if (cmd == '1')
    {
        Scope_SetTimeBase(1);
    }
    else if (cmd == '2')
    {
        Scope_SetTimeBase(2);
    }
    else if (cmd == '3')
    {
        Scope_SetTimeBase(3);
    }
    else if (cmd == 'G')
    {
        scope_requested_command = kCmd_Run;
    }
    else if (cmd == 'S')
    {
        scope_requested_command = kCmd_Stop;
    }
}

// Not used, for testing only

#ifdef SCOPE_TEST_FUNCTIONS
double scope_measure_voltage()
{
   int32_t scope_adc_value = HAL_ADC_GetValue (&hadc1);

   /// Fixed value of analog reference voltage (Vref+), connected to analog voltage supply Vdda (unit: mV).
   /// Measured by multimetter, for tests only.

   const int scope_fixed_adc_vdda_voltage = 3275;
   // Convert adc value to millivolts, test using fixed ref value
   // uint32_t voltage_millivolts = scope_fixed_adc_vdda_voltage * scope_adc_value / scope_adc_max_value;

   // Maximum voltage measurement precision in mV.
   const int adc_precision = 10;

   // Convert adc value to millivolts
   uint32_t voltage_millivolts = __HAL_ADC_CALC_DATA_TO_VOLTAGE(scope_fixed_adc_vdda_voltage, scope_adc_value, ADC_RESOLUTION_12B);
   // Convert to volts with given precision
   double scope_measured_voltage = round ((double) (voltage_millivolts) / adc_precision) * adc_precision;

   return scope_measured_voltage;
}
#endif
