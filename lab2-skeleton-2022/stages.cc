/* rv64-emu -- Simple 64-bit RISC-V simulator
 *
 *    stages.cc - Pipeline stages
 *
 * Copyright (C) 2016-2020  Leiden University, The Netherlands.
 */


#include "stages.h"
#include <iostream>
#include "ControlSignal.h"


// Use Delayslots.
//constexpr bool CPUCFGR_ND = false;

/*
 * Instruction fetch
 */

void
InstructionFetchStage::propagate()
{
  try
  {
    if (IsDelaySlot_)
    {
      ControlSignal_.SetIsDelayInstruction(true);
      IsDelaySlot_ = false;
      IsAfterDelaySlot_ = true;
    }
    else if (IsAfterDelaySlot_)
    {
      ControlSignal_.SetIsDelayInstruction(false);
      MuxPc_.setSelector(PipelineProperties::EPcSelect::PC_Jump);
      IsAfterDelaySlot_ = false;
    }
    PC = MuxPc_.getOutput();
    instructionMemory.setAddress(PC);
    instructionMemory.setSize(4);
    instructionWord = instructionMemory.getValue();

    if (instructionWord == TestEndMarker)
      throw TestEndMarkerEncountered(PC);
  }
  catch (TestEndMarkerEncountered& e)
  {
    if (pipelining)
    {
      HazardDetection_.SetAtEndProgram();
      if (!HazardDetection_.ShouldStall(EStallStages::IF_Propagate))
      {
        throw;
      }
    }
    else
    {
      throw;
    }
  }
  catch (std::exception& e)
  {
    if (pipelining)
    {
      HazardDetection_.SetAtEndProgram();
      if (!HazardDetection_.ShouldStall(EStallStages::IF_Propagate))
      {
        throw InstructionFetchFailure(PC);
      }
    }
    else
    {
      throw InstructionFetchFailure(PC);
    }
  }
}

void
InstructionFetchStage::clockPulse()
{
  if (pipelining)
  {
    if (HazardDetection_.ShouldStall(EStallStages::IF_Pulse))
    {
      if (HazardDetection_.GetIsAtEndProgram())
      {
        // Insert Stall.
        ControlSignal_.SetIsStall(true);
        if_id.ControlSignal_ = ControlSignal_;
        if_id.PC = 0;
        // Nop instruction word.
        if_id.InstructionWord_ = 0b000010101000000000000000000000000;
        return;
      }
      // Keep old pipeline values, due to hazard-stall.
      return;
    }
  }
  if_id.PC = PC;
  if_id.InstructionWord_ = instructionWord;
  ControlSignal_.SetIsStall(false);
  if_id.ControlSignal_ = ControlSignal_;
}


/*
 * Instruction decode
 */

void
dump_instruction(std::ostream& os, const uint32_t instructionWord,
                 const InstructionDecoder& decoder);

void
InstructionDecodeStage::propagate()
{
  ControlSignal_ = if_id.ControlSignal_;
  decoder.SetInstructionWord(if_id.InstructionWord_);
  PC_ = if_id.PC;

  ControlSignal_.SetInstruction(decoder.GetInstruction());
  if (ControlSignal_.NeedImmediate())
  {
    Immediate_ = decoder.GetImmediate(ControlSignal_.GetImmediateProperties());
  }
  if (ControlSignal_.GetIsBranch())
  {
    ControlSignal_.SetBranchAdderValues(&AdderBranch_, PC_, Immediate_);
    if (ControlSignal_.GetIsDelayInstruction())
    {
      throw std::runtime_error("Branch instruction in delay slot");
    }
  }
  Adder_.SetInputA(PC_);
  Adder_.SetInputB(4);
  if (ControlSignal_.NeedA())
  {
    regfile.setRS1(ControlSignal_.GetA());
  }
  if (ControlSignal_.NeedB())
  {
    regfile.setRS2(ControlSignal_.GetB());
  }
  /*
  if (ControlSignal_.NeedD())
  {
    RD = ControlSignal_.GetD();
    regfile.setRD(ControlSignal_.GetD());
  }
  */

  /* debug mode: dump decoded instructions to cerr.
   * In case of no pipelining: always dump.
   * In case of pipelining: special case, if the PC == 0x0 (so on the
   * first cycle), don't dump an instruction. This avoids dumping a
   * dummy instruction on the first cycle when ID is effectively running
   * uninitialized.
   */
  if (debugMode && (! pipelining || (pipelining && PC_ != 0x0)))
  {
    /* Dump program counter & decoded instruction in debug mode */
    auto storeFlags(std::cerr.flags());

    std::cerr << std::hex << std::showbase << PC_ << "\t";
    std::cerr.setf(storeFlags);

    std::cerr << decoder << std::endl;
  }
  if (pipelining)
  {
    HazardDetection_.SetIdControlSignal(ControlSignal_);
    HazardDetection_.SetIDStagePc(PC_);
  }
}

void InstructionDecodeStage::clockPulse()
{
  /* ignore the "instruction" in the first cycle. */
  if (! pipelining || (pipelining && PC_ != 0x0))
    ++nInstrIssued;

  if (pipelining)
  {
    if (HazardDetection_.ShouldStall(EStallStages::ID))
    {
      nStalls++;
      ControlSignal_.SetInstruction(Nop());
      ControlSignal_.SetIsStall(true);
      id_ex.ControlSignal_ = ControlSignal_;
      id_ex.PC = 0;
      id_ex.RS1_ = 0;
      id_ex.RS2_ = 0;
      return;
    }
  }
  ControlSignal_.SetIsStall(false);

  if (ControlSignal_.NeedA())
  {
    id_ex.RS1_ = regfile.getReadData1();
  }
  if (ControlSignal_.NeedB())
  {
    id_ex.RS2_ = regfile.getReadData2();
  }
  // Calculate the new pc values.
  Adder_.CalculateOutput();
  MuxPc_.setInput(PipelineProperties::EPcSelect::PC_Increment, Adder_.Output_);
  if (!ControlSignal_.GetIsDelayInstruction() && ControlSignal_.GetIsBranch())
  {
    // Do not calculate if the instruction is inside a delay slot, we need the previous set jump target.
    AdderBranch_.CalculateOutput();
    MuxPc_.setInput(PipelineProperties::EPcSelect::PC_Jump, AdderBranch_.Output_);
  }
  // Set correct selector signal on Mux.
  switch (ControlSignal_.GetBranchType())
  {
  case Instruction::EBranchType::BT_Immediate:
    IsDelaySlot_ = true;
    break;
  case Instruction::EBranchType::BT_Register:
    MuxPc_.setInput(PipelineProperties::EPcSelect::PC_Jump, id_ex.RS2_);
    IsDelaySlot_ = true;
    break;
  case Instruction::EBranchType::BT_JumpToImmediateOnFlag:
    if (Flag_)
    {
      IsDelaySlot_ = true;
    }
    break;
  case Instruction::EBranchType::BT_JumpToImmediateNoFlag:
    if (!Flag_)
    {
      IsDelaySlot_ = true;
    }
    break;
  case Instruction::EBranchType::BT_SetsFlag:
    Flag_ = ControlSignal_.GetFlagValue(id_ex.RS1_, id_ex.RS2_);
    break;
  case Instruction::EBranchType::BT_Nothing:
    break;
  }
  MuxPc_.setSelector(PipelineProperties::EPcSelect::PC_Increment);

  id_ex.PC = PC_;
  id_ex.Immediate_ = Immediate_;
  id_ex.ControlSignal_ = ControlSignal_;
}

/*
 * Execute
 */

void
ExecuteStage::propagate()
{
  ControlSignal_ = id_ex.ControlSignal_;
  PC_ = id_ex.PC;
  Immediate_ = id_ex.Immediate_;
  RS2_ = id_ex.RS2_;
  Alu_.setOp(ControlSignal_.GetAluInstruction());
  if (ControlSignal_.NeedAlu())
  {
    // Check if we need the PC, immediate or a register.
    Alu_.setA(ControlSignal_.GetAluInputA(&id_ex));
    Alu_.setB(ControlSignal_.GetAluInputB(&id_ex));
  }
}

void
ExecuteStage::clockPulse()
{
  ex_m.ALUOutput_ = Alu_.getResult();
  ex_m.PC = PC_;
  ex_m.Immediate_ = Immediate_;
  ex_m.RS2_ = RS2_;
  ex_m.ControlSignal_ = ControlSignal_;
}

/*
 * Memory
 */

void
MemoryStage::propagate()
{
  ControlSignal_ = ex_m.ControlSignal_;
  PC_ = ex_m.PC;
  RS2_ = ex_m.RS2_;
  ALUOutput_ = ex_m.ALUOutput_;
  Immediate_ = ex_m.Immediate_;

  if (ControlSignal_.NeedReadDataMemory() || ControlSignal_.NeedWriteDataMemory())
  {
    dataMemory.setSize(ControlSignal_.GetDataMemorySize());
    dataMemory.setAddress(ControlSignal_.GetMemAddress(&ex_m));
    if (ControlSignal_.NeedWriteDataMemory())
    {
      dataMemory.setDataIn(ControlSignal_.GetMemWriteValue(&ex_m));
    }
  }
  // Set respective fields to false if no need/write is needed.
  dataMemory.setReadEnable(ControlSignal_.NeedReadDataMemory());
  dataMemory.setWriteEnable(ControlSignal_.NeedWriteDataMemory());
}

void
MemoryStage::clockPulse()
{
  dataMemory.clockPulse();
  if (ControlSignal_.NeedReadDataMemory())
  {
    m_wb.DataMemoryOutput_ = dataMemory.getDataOut(ControlSignal_.NeedSExtDataMemoryOutput());
  }
  dataMemory.setReadEnable(false);
  dataMemory.setWriteEnable(false);
  m_wb.ALUOutput_ = ALUOutput_;
  m_wb.Immediate_ = Immediate_;
  m_wb.ControlSignal_ = ControlSignal_;
  m_wb.PC = PC_;
}

/*
 * Write back
 */

void
WriteBackStage::propagate()
{
  if (! pipelining || (pipelining && m_wb.PC != 0x0))
    ++nInstrCompleted;
  ControlSignal_ = m_wb.ControlSignal_;
  if (ControlSignal_.NeedWriteBackToRd())
  {
    if (ControlSignal_.GetWriteBackType() == Instruction::EWriteBackValueType::Wb_R9ADD)
    {
      regfile.setRD(9);
    }
    else
    {
      regfile.setRD(ControlSignal_.GetD());
    }
    regfile.setWriteEnable(true);
    regfile.setWriteData(ControlSignal_.GetWriteBackValueRD(&m_wb));
  }
  else
  {
    regfile.setWriteEnable(false);
  }
}

void
WriteBackStage::clockPulse()
{
  regfile.clockPulse();
}
