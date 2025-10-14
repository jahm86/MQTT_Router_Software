#ifndef REGS_H
#define REGS_H

// Registros 0x2000 - 0x200E
#define R_ComCtrlCmd 0x2000
#define R_ComSetFreq 0x2001
#define R_PIDRefRange 0x2002
#define R_PIDFbRange 0x2003
#define R_TorqueSetVal 0x2004
#define R_UpperFreqFw 0x2005
#define R_UpperFreqRv 0x2006
#define R_UpperEmTorque 0x2007
#define R_UpperBrkTorque 0x2008
#define R_SpecialCtrlCmd 0x2009
#define R_VirTermCmdI 0x200A
#define R_VirTermCmdO 0x200B
#define R_VSetValue 0x200C
#define R_AOSet1 0x200D
#define R_AOSet2 0x200E
#define GROUP_A_FIRST (R_ComCtrlCmd)
#define GROUP_A_LEN (R_AOSet2 - R_ComCtrlCmd + 1)

// Registros 0x2100 - 0x2103
#define R_SW1A 0x2100
#define R_SW1B 0x2101
#define R_InvFaultCode 0x2102
#define R_VFDID 0x2103
#define GROUP_B_FIRST (R_SW1A)
#define GROUP_B_LEN (R_VFDID - R_SW1A + 1)

// Registros 0x3000 - 0x3016
#define R_OpFreq 0x3000
#define R_SetFreq 0x3001
#define R_VBus 0x3002
#define R_VOut 0x3003
#define R_IOut 0x3004
#define R_OpSpeed 0x3005
#define R_PwrOut 0x3006
#define R_TorqueOut 0x3007
#define R_CLoopSet 0x3008
#define R_CLoopFb 0x3009
#define R_PIDSet 0x3008
#define R_PIDFb 0x3009
#define R_IOInA 0x300A
#define R_IOInB 0x300B
#define R_AI1 0x300C
#define R_AI2 0x300D
#define R_AI3 0x300E
#define R_AI4 0x300F
#define R_HSPulseI1 0x3010
#define R_HSPulseI2 0x3011
#define R_StepSpd 0x3012
#define R_ExtLen 0x3013
#define R_ExtCount 0x3014
#define R_TorqueSet 0x3015
#define R_InvCode 0x3016
#define GROUP_C_FIRST (R_OpFreq)
#define GROUP_C_LEN (R_InvCode - R_OpFreq + 1)

// Registro 0x5000
#define R_FaultCode 0x5000
#define GROUP_D_FIRST (R_FaultCode)
#define GROUP_D_LEN (1)

#endif // REGS_H
