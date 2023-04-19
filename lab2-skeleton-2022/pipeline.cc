/* rv64-emu -- Simple 64-bit RISC-V simulator
 *
 *    pipeline.cc - Classic 5-stage RISC pipeline
 *
 * Copyright (C) 2016-2021  Leiden University, The Netherlands.
 */

#include "pipeline.h"


Pipeline::Pipeline(bool pipelining,
                   bool debugMode,
                   MemAddress& PC,
                   InstructionMemory& instructionMemory,
                   InstructionDecoder& decoder,
                   RegisterFile& regfile,
                   Mux<MemAddress, PipelineProperties::EPcSelect>& pcSelect,
                   bool& flag,
                   bool& isDelaySlot,
                   ControlSignal controlSignal,
                   DataMemory& dataMemory)
  : pipelining{pipelining},
    HazardDetection_(if_id, id_ex, ex_m, m_wb)
{

  stages.emplace_back(std::make_unique<InstructionFetchStage>(pipelining,
                                                              if_id,
                                                              instructionMemory,
                                                              PC,
                                                              pcSelect,
                                                              isDelaySlot,
                                                              controlSignal,
                                                              HazardDetection_));
  stages.emplace_back(std::make_unique<InstructionDecodeStage>(pipelining,
                                                               if_id, id_ex,
                                                               regfile,
                                                               decoder,
                                                               controlSignal,
                                                               nInstrIssued,
                                                               nStalls,
                                                               pcSelect,
                                                               flag,
                                                               isDelaySlot,
                                                              HazardDetection_,
                                                               debugMode));
  stages.emplace_back(std::make_unique<ExecuteStage>(pipelining,
                                                     id_ex, ex_m,
                                                     controlSignal,
                                                     ALU_,
                                                     HazardDetection_));
  stages.emplace_back(std::make_unique<MemoryStage>(pipelining,
                                                    ex_m, m_wb,
                                                    dataMemory,
                                                    controlSignal,
                                                    HazardDetection_));
  stages.emplace_back(std::make_unique<WriteBackStage>(pipelining,
                                                       m_wb,
                                                       regfile,
                                                       nInstrCompleted,
                                                       controlSignal,
                                                       HazardDetection_));
}

void
Pipeline::propagate()
{
  if (! pipelining)
  {
    /* Execute a single instruction execution step. */
    stages[currentStage]->propagate();
  }
  else
  {
    /* Run propagate for all stages within a single clock cycle. */
    for (auto& s : stages)
      s->propagate();
  }
}

void
Pipeline::clockPulse()
{
  if (! pipelining)
  {
    stages[currentStage]->clockPulse();
    currentStage = (currentStage + 1) % stages.size();
  }
  else
  {
    for (auto& s : stages)
      s->clockPulse();
  }
}
