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
/// Increasing this number gives more time for processing in ADC interrupt but reduces the output frame rate.
/// The output frame rate shoudn't go below the maximum frame rate supported by transport / UART.
#define SCOPE_ADC_FRAMES_IN_HALF_BUFFER         (5)

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

#define SCOPE_OUT_MAX_FRAME_DATA_SIZE   (SCOPE_ADC_MAX_FRAME_SIZE * 2)  // Maximum allowed OUTPUT frame data size in bytes
#define SCOPE_OUT_MAX_FRAME_SIZE        (SCOPE_OUT_MAX_FRAME_DATA_SIZE + SCOPE_OUT_FRAME_HEADER_SIZE)  // Maximu allowed OUTPUT frame size in bytes (with header)
#define SCOPE_OUT_MAX_DMA_BUFFER_SIZE    SCOPE_OUT_MAX_FRAME_SIZE  // We send single frame at a time

#define SCOPE_DEFAULT_TIME_BASE         kTimeBase_10ms

/// Maximum value of ADC conversion for 12 bit adc resolution
#define SCOPE_ADC_MAX_VALUE __LL_ADC_DIGITAL_SCALE(LL_ADC_RESOLUTION_12B) // 4095


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


/// Indicates the state of ADC acquisition and copying of a single frame of data from the ADC ring-buffer.
enum FrameAcquisitionStatus
{
    /// ADC sampler is not working
    kFrameAcquisitionStatus_Idle,
    /**
    Data acquisition and frame copy is done, frame is ready to be transmitted.
    If there is transmission in progress, this frame won't be transmitted, will be skipped.
    */
    kFrameAcquisitionStatus_Completed,
    /// Data acquisition of the left buffer is over and frame is being copied.
    kFrameAcquisitionStatus_LeftHalf,
    ///  Data acquisition of the right buffer is over and frame is being copied.
    kFrameAcquisitionStatus_RightHalf
};

/// Indicates to the transmitter whether the output frame is ready to be send or not
enum OutputFrameReadyStatus
{
    /// ADC sampler is not working
    kOutputFrameReadyStatus_Idle,
    /// Waiting for new frame to be available in the frame output buffer.
    kOutputFrameReadyStatus_Waiting,
    /**
    New frame is ready in the frame output buffer.
    It is set after kFrameAcquisitionStatus_Completed if there is no ongoing transmission.
    If there is transmission in progress, this frame won't be transmitted, will be skipped.
    */
    kOutputFrameReadyStatus_Ready,
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

enum ScopeCommand scope_requested_command = kCmd_None;

uint16_t scope_adc_vdda_voltage;
size_t scope_part_out_frame_data_size;
enum TimeBaseCode scope_requested_new_time_base_code;
enum TimeBaseCode scope_time_base_code = SCOPE_DEFAULT_TIME_BASE;

// ADC DMA ring-buffer.
// This buffer records a continues stream of ADC samples in real-time.

uint16_t scope_adc_dma_buffer[SCOPE_ADC_MAX_DMA_BUFFER_SIZE];

uint16_t scope_adc_frame_size;
uint16_t scope_adc_dma_buffer_half_size;
uint16_t scope_adc_dma_buffer_size;

uint16_t* scope_adc_dma_buffer_left_begin;
uint16_t* scope_adc_dma_buffer_left_end;
uint16_t* scope_adc_dma_buffer_right_begin;
uint16_t* scope_adc_dma_buffer_right_end;

// Output frame double-buffer.

uint8_t scope_out_frame_buffer_1[SCOPE_OUT_MAX_DMA_BUFFER_SIZE];
uint8_t scope_out_frame_buffer_2[SCOPE_OUT_MAX_DMA_BUFFER_SIZE];

uint8_t scope_out_encoded_frame_buffer[SCOPE_OUT_MAX_DMA_BUFFER_SIZE];
uint16_t scope_out_encoded_frame_buffer_size = 0;

uint8_t* scope_out_frame_front_buffer = scope_out_frame_buffer_1;
uint8_t* scope_out_frame_back_buffer = scope_out_frame_buffer_2;

uint16_t scope_out_frame_data_size;     // frame size without header
uint16_t scope_out_frame_buffer_size;   // frame full size

enum RunState scope_run_state = kRunState_Invalid;

enum ScopeEdgeType scope_edge_type = kScopeEdge_Rising;
enum ScopeEdgeType scope_requested_edge_type = kScopeEdge_Rising;

enum ScopeTriggerType scope_trigger_type = kScopeTrigger_Auto;
enum ScopeTriggerType scope_requested_trigger_type = kScopeTrigger_Auto;

enum FrameAcquisitionStatus scope_adc_frame_status = kFrameAcquisitionStatus_Idle;
enum FrameAcquisitionStatus scope_adc_frame_status_once = kFrameAcquisitionStatus_Idle;
enum OutDataTxStatus scope_out_data_tx_status = kOutDataTxStatus_Idle;
enum OutputFrameReadyStatus scope_out_frame_ready_status = kOutputFrameReadyStatus_Idle;


uint16_t scope_default_trigger_value = SCOPE_ADC_MAX_VALUE / 2;

// ------------- local scope functions ---------------

static
bool scope_is_time_base_code_valid(enum TimeBaseCode timeBaseCode)
{
    return timeBaseCode > kTimeBase_Invalid && timeBaseCode < kTimeBase_End;
}

static
void scope_reset_status()
{
    scope_adc_frame_status = kFrameAcquisitionStatus_Idle;
    scope_adc_frame_status_once = kFrameAcquisitionStatus_Idle;
    scope_out_frame_ready_status = kOutputFrameReadyStatus_Idle;
    scope_part_out_frame_data_size = 0;
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

    scope_adc_dma_buffer_half_size = scope_adc_frame_size * SCOPE_ADC_FRAMES_IN_HALF_BUFFER;
    scope_adc_dma_buffer_size = scope_adc_dma_buffer_half_size * 2;

    scope_adc_dma_buffer_left_begin = scope_adc_dma_buffer;

    scope_adc_dma_buffer_left_end = scope_adc_dma_buffer_left_begin + scope_adc_dma_buffer_half_size;
    scope_adc_dma_buffer_right_begin = scope_adc_dma_buffer_left_end;
    scope_adc_dma_buffer_right_end = scope_adc_dma_buffer_right_begin + scope_adc_dma_buffer_half_size;

    scope_out_frame_data_size = scope_adc_frame_size * SCOPE_ADC_SAMPLE_SIZE;
    scope_out_frame_buffer_size = scope_out_frame_data_size + SCOPE_OUT_FRAME_HEADER_SIZE;
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
    memset((char*)scope_adc_dma_buffer, 0, SCOPE_ADC_MAX_DMA_BUFFER_SIZE * 2);
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
    // This method is called from interrupt and works in synch with frame transmission method.

    // Cannot swap output buffer if transmission is in progress.
    // Output frame will be lost.
    if (scope_out_data_tx_status != kOutDataTxStatus_InProgress)
    {
        // Swap buffers.
        uint8_t* tmp = scope_out_frame_front_buffer;
        scope_out_frame_front_buffer = scope_out_frame_back_buffer;
        scope_out_frame_back_buffer = tmp;

        scope_out_frame_ready_status = kOutputFrameReadyStatus_Ready;
    }

    assert (scope_out_frame_front_buffer != scope_out_frame_back_buffer);

    // Update acquisition parameters.
    scope_edge_type = scope_requested_edge_type;
    scope_trigger_type = scope_requested_trigger_type;

    // Frame acquisition process is now completed.
    scope_adc_frame_status = kFrameAcquisitionStatus_Completed;
}

static
bool scope_copy_frame_to_backbuffer(const uint16_t* adc_frame_begin)
{
    uint8_t* const out_buffer_begin = scope_out_frame_back_buffer + SCOPE_OUT_FRAME_HEADER_SIZE;

    if (scope_part_out_frame_data_size > 0)
    {
        // Step 2 of output buffer collection process.
        // Copy remaining adc data to output buffer.
        const size_t first_part_out_frame_data_size = scope_out_frame_data_size - scope_part_out_frame_data_size;

        assert (adc_frame_begin == scope_adc_dma_buffer_left_begin || adc_frame_begin == scope_adc_dma_buffer_left_end);
        assert (scope_part_out_frame_data_size + first_part_out_frame_data_size == scope_out_frame_data_size);

        memcpy(out_buffer_begin + first_part_out_frame_data_size, adc_frame_begin, scope_part_out_frame_data_size);
        scope_part_out_frame_data_size = 0;
        return true;
    }

    const uint16_t* const adc_frame_end = adc_frame_begin + scope_adc_frame_size;

    if (adc_frame_begin < scope_adc_dma_buffer_left_end)
    {
        if (adc_frame_end <= scope_adc_dma_buffer_left_end)
        {
            memcpy(out_buffer_begin, adc_frame_begin, scope_out_frame_data_size);
            scope_part_out_frame_data_size = 0;
            return true;
        }
        else if (adc_frame_end > scope_adc_dma_buffer_left_end)
        {
            // Step 1 of output buffer collection process.
            // Remaining data will be copied when copy_frame_to_backbuffer() is called in next adc interrupt
            scope_part_out_frame_data_size = sizeof(uint16_t) * (adc_frame_end - scope_adc_dma_buffer_left_end);
            assert(scope_out_frame_data_size > scope_part_out_frame_data_size);
            memcpy(out_buffer_begin, adc_frame_begin, scope_out_frame_data_size - scope_part_out_frame_data_size);
            return false;
        }
    }

    else if (adc_frame_begin >= scope_adc_dma_buffer_left_end)
    {
        if (adc_frame_end <= scope_adc_dma_buffer_right_end)
        {
            memcpy(out_buffer_begin, adc_frame_begin, scope_out_frame_data_size);
            scope_part_out_frame_data_size = 0;
            return true;
        }
        else if (adc_frame_end > scope_adc_dma_buffer_right_end)
        {
            // Step 1 of output buffer collection process.
            // Remaining data will be copied when copy_frame_to_backbuffer() is called in next adc interrupt
            const size_t part_frame_size = adc_frame_end - scope_adc_dma_buffer_right_end;
            scope_part_out_frame_data_size = sizeof(uint16_t) * part_frame_size;
            assert(scope_out_frame_data_size > scope_part_out_frame_data_size);
            memcpy(out_buffer_begin, adc_frame_begin, scope_out_frame_data_size - scope_part_out_frame_data_size);
            return false;
        }
    }

    // we never get here
    assert(false);
    scope_part_out_frame_data_size = 0;
    return true;
}

static
const uint16_t* scope_find_trigger_in_sample()
{
    //scope_find_trigger_in_sample_active = true;

    const uint16_t* adc_trigger_pointer = 0;

    uint16_t* begin;
    uint16_t* end;

    // Out adc dma buffer is logically divided in 2 parts,
    // assign pointers to the half of the buffer that is currently accessible for reading

    if (scope_adc_frame_status == kFrameAcquisitionStatus_LeftHalf)
    {
        begin = scope_adc_dma_buffer_left_begin;
        end = scope_adc_dma_buffer_left_end;
    }
    else if (scope_adc_frame_status == kFrameAcquisitionStatus_RightHalf)
    {
        begin = scope_adc_dma_buffer_right_begin;
        end = scope_adc_dma_buffer_right_end;
    }
    else
    {
        assert(false);
    }


    // Auto find trigger - use mean signal value as trigger value

    uint16_t min_value = -1;
    uint16_t max_value = 0;

    for (uint16_t* it = begin; it < end; ++it)
    {
        const uint16_t value = *it;
        min_value = MIN(min_value, value);
        max_value = MAX(max_value, value);
    }

    const uint16_t mean_value = (max_value - min_value + 1) >> 1;
    const uint16_t scope_trigger_value = min_value + mean_value;


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

/**
 * Return a pointer to left or right part of ADC buffer, depending on the adc status enum.
 */
static
uint16_t* scope_adc_dma_buffer_begin()
{
    switch (scope_adc_frame_status)
    {
    case kFrameAcquisitionStatus_LeftHalf:
        return scope_adc_dma_buffer_left_begin;
    case kFrameAcquisitionStatus_RightHalf:
        return scope_adc_dma_buffer_right_begin;
    case kFrameAcquisitionStatus_Completed:
        assert(false);
    default:
        assert(false);
        return scope_adc_dma_buffer_left_begin;
    };
}

/**
 * Copies adc frame to back-buffer.
 * If adc frames spans over the adc buffer halves, the frame is copied in 2 steps,
 * where other part of the frame is copied during second call to this method.
 * Swaps back-buffer with front-buffer when copy is completed
 * front-buffer is then ready to be transmitted.
 * This method should be called in ADC ConvCpltCallback and ADC ConvHalfCpltCallback.
 * The adcDataStatus enum tells which part of the ADC buffer is ready (left or right).
 * The scope_adc_frame_status is then set to this value and changed to kFrameAcquisitionStatus_Completed only after
 * adc frame is fully copied.
 */
static
void scope_on_adc_data_ready(enum FrameAcquisitionStatus adcDataStatus)
{
    // Scope_adc_data_status_once is to detect the situation where
    // buffer is filled for the first time and we may not have the data in the right buffer yet.
    // scope_adc_frame_status_once is then kFrameAcquisitionStatus_LeftHalf.
    // Once the right buffer is filled it changes to kFrameAcquisitionStatus_RightHalf.
    if (scope_adc_frame_status_once == kFrameAcquisitionStatus_Idle)
    {
        scope_adc_frame_status_once = adcDataStatus;
    }

    if (scope_adc_frame_status == kFrameAcquisitionStatus_Completed ||
        scope_adc_frame_status == kFrameAcquisitionStatus_Idle)
    {
        // Previous copy operation was completed or this is the first copy.
        // We find trigger position in the ADC buffer, this will mark the start of ADC frame to copy.
        // In trigger 'none' mode, frame always starts at the beginning of the ADC buffer.
        scope_adc_frame_status = adcDataStatus;
        const uint16_t* adcFramePointer = 0;
        if (scope_trigger_type == kScopeTrigger_Auto)
        {
           adcFramePointer = scope_find_trigger_in_sample();
        }
        if (adcFramePointer == 0)
        {
            // In trigger 'none' mode or if trigger was not found, just point to start of the buffer
            adcFramePointer = scope_adc_dma_buffer_begin();
        }
        assert (adcFramePointer != 0);
        if (adcFramePointer != 0)
        {
            const bool complete_frame_copied = scope_copy_frame_to_backbuffer(adcFramePointer);
            if (complete_frame_copied)
            {
                scope_swap_out_buffers();
            }
            else
            {
                // Step 1 of partial frame copy is completed.
                // There is still a missing part of frame to be copied.
                // It will be copied on the next adc interrupt, next time on_adc_data_ready() is called
            }
        }
    }
    else if (scope_adc_frame_status != adcDataStatus)
    {
        assert(adcDataStatus == kFrameAcquisitionStatus_LeftHalf || adcDataStatus == kFrameAcquisitionStatus_RightHalf);
        assert(scope_trigger_type == kScopeTrigger_Auto);

        // Step 2 of partial frame copy in trigger mode.
        // Only called in trigger mode
        scope_adc_frame_status = adcDataStatus;
        const bool complete_frame_copied = scope_copy_frame_to_backbuffer(scope_adc_dma_buffer_begin());
        if (complete_frame_copied)
        {
            scope_swap_out_buffers();
        }
        else
        {
            // The second copy finishes the job, should always be successful.
            assert(false);
        }
    }
    else
    {
        // Called twice with the same scope_adc_frame_status.
        // We must have skipped other part of buffer.
        assert(false);
    }
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

/// Right ADC buffer
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        scope_on_adc_data_ready(kFrameAcquisitionStatus_RightHalf);
    }
}

/// Left ADC buffer
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        scope_on_adc_data_ready(kFrameAcquisitionStatus_LeftHalf);
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

    // Encode / pack output frame to make it 20% smaller.
    // Each 5 12bit samples are packed/encoded into 4 samples by encoding 5th 12 bit sample
    // into free 3-bit space in 4 12bit samples (12,12,12,12,12 - > 12 + 3, 12 + 3, 12 + 3, 12 + 3).
    // The encoded 4 sample pack contains data in first 15 bits.
    // Last byte is reserved for special codes (like frame start).

    const uint16_t src_pack_size_samples = 5;
    const uint16_t dest_pack_size_samples = src_pack_size_samples - 1;
    const uint16_t src_pack_size = src_pack_size_samples * SCOPE_ADC_SAMPLE_SIZE;
    const uint16_t dest_pack_size = dest_pack_size_samples * SCOPE_ADC_SAMPLE_SIZE;
    const uint16_t pack_count = scope_out_frame_data_size / src_pack_size;
    const uint16_t tail_data_size = scope_out_frame_data_size % src_pack_size;

    // The data that will be encoded
    const uint16_t src_encoded_data_size = pack_count * src_pack_size_samples;
    assert(src_encoded_data_size * SCOPE_ADC_SAMPLE_SIZE  + tail_data_size == scope_out_frame_data_size);

    // The encoded (packed) data.
    const uint16_t dest_encoded_data_size = pack_count * dest_pack_size_samples;
    scope_out_encoded_frame_buffer_size = SCOPE_OUT_FRAME_HEADER_SIZE + dest_encoded_data_size * SCOPE_ADC_SAMPLE_SIZE + tail_data_size;

    // Source range to be encoded
    const uint16_t* src_begin = scope_out_frame_front_buffer + SCOPE_OUT_FRAME_HEADER_SIZE;
    const uint16_t* src_end = src_begin + src_encoded_data_size;

    // Destination range
    uint16_t* dest = scope_out_encoded_frame_buffer + SCOPE_OUT_FRAME_HEADER_SIZE;
    uint16_t* dest_end = dest + dest_encoded_data_size;

    // Encode
    for (uint16_t* src = src_begin; src < src_end; src++)
    {
        const uint16_t* src_last_sample = (uint16_t*)(src + dest_pack_size_samples);
        const uint16_t last_sample = *src_last_sample;
        // 3 bits from last sample are added at bit 12 - bit 14 positions 0b0xxxnnnnnnnnn
        // bit 15 is reserved for special codes like start of frame (0xFFFE)
        *dest++ = *src++ | ((last_sample & 0b0000000000000111) << 12);
        *dest++ = *src++ | ((last_sample & 0b0000000000111000) << 9);
        *dest++ = *src++ | ((last_sample & 0b0000000111000000) << 6);
        *dest++ = *src++ | ((last_sample & 0b0000111000000000) << 3);
    }

    assert (dest == dest_end);

    // The tail data are not encoded, just copy it
    if (tail_data_size > 0)
    {
        memcpy (dest_end, src_end, tail_data_size);
    }
}


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
    assert(scope_out_data_tx_status != kOutDataTxStatus_InProgress);

    // Wait for kOutputFrameReadyStatus_Ready unless acquisition is stopped.
    // Only start transmitting new frame if new complete frame is available in the frame output buffer
    // While waiting for frame, we can run all requested commands that must be executed only between frames.
    // If acquisition is stopped, don't wait, continuously transmit last frame.
    do
    {
        scope_process_requested_commands();
        if (scope_run_state == kRunState_Stopped)
        {
            break;
        }
    } while(scope_out_frame_ready_status != kOutputFrameReadyStatus_Ready);

    // Disable output buffer swap.
    // Buffer swap will be re-enabled when transmission is finished.
    scope_out_data_tx_status = kOutDataTxStatus_InProgress;

    // Reset the OutputFrameReadyStatus,
    // Next call to scope_swap_out_buffers() will set the status to kOutputFrameReadyStatus_Ready.
    scope_out_frame_ready_status = kOutputFrameReadyStatus_Waiting;

    uint32_t tickstart = HAL_GetTick();

    // Pack (scope_out_frame_front_buffer, scope_out_frame_buffer_size)
    // to (scope_out_encoded_frame_buffer, scope_out_encoded_frame_buffer_size)
    // Packed output frame is 20% smaller.
    scope_encode_output_frame();

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&hlpuart1, scope_out_encoded_frame_buffer, scope_out_encoded_frame_buffer_size);

    UNUSED(status);

    // Wait for the UART DMA to finish the frame transmission (wait for HAL_UART_TxCpltCallback())
    const uint32_t txTimeout = 1000;
    while (scope_out_data_tx_status != kOutDataTxStatus_Completed && (HAL_GetTick() - tickstart) < txTimeout);

    if (scope_out_data_tx_status != kOutDataTxStatus_Completed)
    {
        // Timeout occurred.
        // Force status kOutDataTxStatus_Completed in case of timeout.
        scope_out_data_tx_status = kOutDataTxStatus_Completed;
    }

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
