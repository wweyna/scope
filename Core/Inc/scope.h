/*
 * cope.h
 *
 *  Created on: Nov 11, 2024
 *      Author: Wojciech Weyna
 */

#ifndef INC_SCOPE_H_
#define INC_SCOPE_H_

/// Signal edge types
enum ScopeEdgeType
{
    kScopeEdge_Rising,
    kScopeEdge_Falling
};

/// Trigger types
enum ScopeTriggerType
{
    kScopeTrigger_Auto,
    kScopeTrigger_None
};

/// Time bases supported by scope
enum TimeBaseCode
{
    kTimeBase_Invalid = 0,
    kTimeBase_10ms = 1,
    kTimeBase_1ms = 2,
    kTimeBase_250ms = 3,
    kTimeBase_End
};

void Scope_Run();
void Scope_Stop();
void Scope_TransmitFrame();
void Scope_OnCommandReceived(uint8_t cmd);
void Scope_SetTimeBase(enum TimeBaseCode timeBase);
void Scope_SetEdge(enum ScopeEdgeType edgeType);
void Scope_SetTrigger(enum ScopeTriggerType triggerType);

#endif
