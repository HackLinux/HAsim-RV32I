`include "awb/provides/matrix_multiply_common.bsh"

import Complex::*;
import FixedPoint::*;

typedef Bit#(64) CYCLE_COUNTER;

typedef `MATRIX_MULTIPLY_MAX_SIZE_X MATRIX_X_MAX;
typedef `MATRIX_MULTIPLY_MAX_SIZE_Y MATRIX_Y_MAX;
typedef Bit#(TLog#(MATRIX_X_MAX)) MATRIX_ADDR_X_MAX;
typedef Bit#(TLog#(MATRIX_Y_MAX)) MATRIX_ADDR_Y_MAX;
typedef Bit#(TAdd#(TLog#(TLog#(MATRIX_X_MAX)), 1)) MATRIX_SIZE_BITS;

typedef `MATRIX_MULTIPLY_NUM_ENGINES N_ENGINES;
typedef `MATRIX_MULTIPLY_NUM_LOCAL_MULTIPLIERS N_LOCAL_MULTIPLIERS;

typedef TMul#(MATRIX_X_MAX, MATRIX_Y_MAX)  MATRIX_MAX_NUM_ELEMENTS;
typedef Bit#(TAdd#(TLog#(MATRIX_MAX_NUM_ELEMENTS), 1)) MEM_ADDRESS;

// definitions for matrix multiply instructions
typedef union tagged 
{
    struct{
        Bit#(16) engineNum;
        Bit#(8)  matrixSizeA;
        Bit#(8)  matrixSizeB;
        Bit#(8)  blockSize;
    } INIT_INST;

    struct{
        Bit#(16) engineID;
        Bit#(24) startAx;
        Bit#(24) startAy;
        Bit#(24) startBx;
        Bit#(24) startBy;
        Bit#(8)  blockInfo;
    } COMPUTE_INST;

    struct{
        Bit#(8) logElement;
    } CHECK_INST;
} 
MATRIX_MULTIPLY_INSTRUCTION
    deriving (Bits, Eq);

typedef struct
{
    Bool isFirstBlock;
    Bool isLastBlock;
    Bool needWriteBack;
    Bool newBlock;
    Bool isLastInst;
}
MATRIX_MULTIPLY_BLOCK_INFO
    deriving (Bits, Eq);

// definitions for matrix-multiply engines
typedef `MATRIX_MULTIPLY_BLOCK_SIZE BLOCK_SIZE;
typedef Bit#(TAdd#(TLog#(TLog#(BLOCK_SIZE)), 1)) BLOCK_SIZE_BITS;
typedef   4 MATRIX_MULTIPLY_CMD_FIFO_DEPTH;
typedef 256 MATRIX_A_BUFFER_DEPTH;
typedef  32 MATRIX_B_BUFFER_DEPTH;
typedef TMul#(BLOCK_SIZE,BLOCK_SIZE) MATRIX_C_WRITE_BACK_BUFFER_DEPTH;
typedef  4  MATRIX_C_WRITE_BACK_FIFO_DEPTH;

// definitions for multi-fpga settings
typedef `MATRIX_MULTIPLY_NUM_PARTITIONS N_PARTITIONS;
typedef TDiv#(N_ENGINES, N_PARTITIONS) N_ENGINES_PER_PARTITION;
typedef N_ENGINES_PER_PARTITION N_LOCAL_ENGINES;
typedef TMul#(N_ENGINES_PER_PARTITION, n) REMOTE_START_ENGINE#(numeric type n);

// definitions of test data type
// typedef Bit#(32) TEST_DATA;
typedef Complex#(Bit#(16)) TEST_DATA;


