`include "miriscv_defines.v"

module miriscv_core (
  // clock, reset
  input clk_i,
  input arstn_i,

  // instruction memory interface
  input         [31:0]  instr_rdata_i,
  output logic  [31:0]  instr_addr_o,

  // data memory interface
  input         [31:0]  data_rdata_i,
  output  logic         data_req_o,
  output  logic         data_we_o,
  output  logic [3:0]   data_be_o,
  output  logic [31:0]  data_addr_o,
  output  logic [31:0]  data_wdata_o,

  // RVFI signals
  output  logic         rvfi_valid_o,
  output  logic [63:0]  rvfi_order_o,
  output  logic [31:0]  rvfi_insn_o,
  output  logic         rvfi_trap_o,
  output  logic         rvfi_halt_o,
  output  logic         rvfi_intr_o,
  output  logic [1:0]   rvfi_mode_o,
  output  logic [1:0]   rvfi_ixl_o,
  output  logic [4:0]   rvfi_rs1_addr_o,
  output  logic [4:0]   rvfi_rs2_addr_o,
  output  logic [31:0]  rvfi_rs1_rdata_o,
  output  logic [31:0]  rvfi_rs2_rdata_o,
  output  logic [4:0]   rvfi_rd_addr_o,
  output  logic [31:0]  rvfi_rd_wdata_o,
  output  logic [31:0]  rvfi_pc_rdata_o,
  output  logic [31:0]  rvfi_pc_wdata_o,
  output  logic [31:0]  rvfi_mem_addr_o,
  output  logic [3:0]   rvfi_mem_rmask_o,
  output  logic [3:0]   rvfi_mem_wmask_o,
  output  logic [31:0]  rvfi_mem_rdata_o,
  output  logic [31:0]  rvfi_mem_wdata_o
);

  // === Signal declarations ===
  // Fetch
  logic [31:0]  cu_pc_addr;
  logic [31:0]  fetched_instr;
  logic         cu_instr_valid;

  // Decode
  logic [`ALU_OP_WIDTH-1:0] d_ex_alu_op;
  logic [1:0]               d_ex_op1_sel;
  logic [2:0]               d_ex_op2_sel;
  logic                     d_mem_we;
  logic [2:0]               d_mem_size;
  logic                     d_mem_req;
  logic                     d_wb_src_sel;
  logic                     d_gpr_rd_we;
  logic                     d_illegal_instr;
  logic                     d_branch;
  logic                     d_jal;
  logic                     d_jalr;

  // Immediates
  logic [31:0]  imm_i;
  logic [31:0]  imm_u;
  logic [31:0]  imm_s;
  logic [31:0]  imm_b;
  logic [31:0]  imm_j;

  // General purpose registers
  logic [4:0]   rs1_addr;
  logic [31:0]  rs1_data;
  logic [4:0]   rs2_addr;
  logic [31:0]  rs2_data;
  logic [4:0]   rd_addr;
  logic [31:0]  rd_data;
  logic         rd_we;

  // Execute
  logic [31:0]  d_ex_op1;
  logic [31:0]  d_ex_op2;
  logic [31:0]  ex_result;
  logic         ex_branch;

  // Load-store unit
  logic         mem_we;
  logic [2:0]   mem_size;
  logic         mem_req;
  logic [31:0]  mem_addr;
  logic [31:0]  mem_wdata;
  logic [31:0]  mem_rdata;
  logic         mem_kill;
  logic         mem_stall_req;
  logic         cu_lsu_kill;

  // Writeback
  logic [31:0]  wb_rd_data;


  assign instr_addr_o  = cu_pc_addr;
  assign fetched_instr = instr_rdata_i;

  miriscv_decode decode (
    .fetched_instr_i  ( fetched_instr   ),
    .instr_valid_i    ( cu_instr_valid  ),
    .alu_op_o         ( d_ex_alu_op     ),
    .ex_op_a_sel_o    ( d_ex_op1_sel    ),
    .ex_op_b_sel_o    ( d_ex_op2_sel    ),
    .mem_we_o         ( d_mem_we        ),
    .mem_size_o       ( d_mem_size      ),
    .mem_req_o        ( d_mem_req       ),
    .wb_src_sel_o     ( d_wb_src_sel    ),
    .gpr_we_a_o       ( d_gpr_rd_we     ),
    .illegal_instr_o  ( d_illegal_instr ),
    .branch_o         ( d_branch        ),
    .jal_o            ( d_jal           ),
    .jalr_o           ( d_jalr          )
  );

  assign rd_we = d_gpr_rd_we && ~mem_stall_req;

  assign rs1_addr = fetched_instr[19:15];
  assign rs2_addr = fetched_instr[24:20];
  assign rd_addr  = fetched_instr[11:7];

  miriscv_register_file #(
    .DATA_WIDTH ( 32 )
  ) gpr (
    .clk_i      ( clk_i    ),
    .rst_n_i    ( arstn_i  ),
    .raddr_a_i  ( rs1_addr ),
    .rdata_a_o  ( rs1_data ),
    .raddr_b_i  ( rs2_addr ),
    .rdata_b_o  ( rs2_data ),
    .waddr_a_i  ( rd_addr  ),
    .wdata_a_i  ( rd_data  ),
    .we_a_i     ( rd_we    )
  );

  assign imm_i = { {21{fetched_instr[31]}}, fetched_instr[30:20] };
  assign imm_u = { fetched_instr[31:12], 12'b0 };
  assign imm_s = { {21{fetched_instr[31]}}, fetched_instr[30:25], fetched_instr[11:7] };
  assign imm_b = { {20{fetched_instr[31]}}, fetched_instr[7], fetched_instr[30:25], fetched_instr[11:8], 1'b0 };
  assign imm_j = { {12{fetched_instr[31]}}, fetched_instr[19:12], fetched_instr[20], fetched_instr[30:21], 1'b0 };


  always_comb begin
    unique case ( d_ex_op1_sel )
      `OP_A_RS1:     d_ex_op1 = rs1_data;
      `OP_A_CURR_PC: d_ex_op1 = cu_pc_addr;
      `OP_A_ZERO:    d_ex_op1 = 32'b0;
    endcase
  end

  always_comb begin
    unique case ( d_ex_op2_sel )
      `OP_B_RS2:   d_ex_op2 = rs2_data;
      `OP_B_IMM_I: d_ex_op2 = imm_i;
      `OP_B_IMM_U: d_ex_op2 = imm_u;
      `OP_B_IMM_S: d_ex_op2 = imm_s;
      `OP_B_INCR:  d_ex_op2 = 32'h4;
    endcase
  end

  miriscv_alu alu (
    .operator_i           ( d_ex_alu_op ),
    .operand_a_i          ( d_ex_op1    ),
    .operand_b_i          ( d_ex_op2    ),
    .result_o             ( ex_result   ),
    .comparison_result_o  ( ex_branch   )
  );

  assign mem_we    = d_mem_we;
  assign mem_size  = d_mem_size;
  assign mem_req   = d_mem_req;
  assign mem_addr  = ex_result;
  assign mem_wdata = rs2_data;

  miriscv_lsu lsu (
    .clk_i                  ( clk_i         ),
    .arstn_i                ( arstn_i       ),
    .data_rdata_i           ( data_rdata_i  ),
    .data_req_o             ( data_req_o    ),
    .data_we_o              ( data_we_o     ),
    .data_be_o              ( data_be_o     ),
    .data_addr_o            ( data_addr_o   ),
    .data_wdata_o           ( data_wdata_o  ),
    .lsu_addr_i             ( mem_addr      ),
    .lsu_we_i               ( mem_we        ),
    .lsu_size_i             ( mem_size      ),
    .lsu_data_i             ( mem_wdata     ),
    .lsu_req_i              ( mem_req       ),
    .lsu_kill_i             ( mem_kill      ),
    .lsu_stall_req_o        ( mem_stall_req ),
    .lsu_data_o             ( mem_rdata     )
  );

  assign rd_data = wb_rd_data;

  always_comb begin
    unique case ( d_wb_src_sel )
      `WB_EX_RESULT: wb_rd_data = ex_result;
      `WB_LSU_DATA:  wb_rd_data = mem_rdata;
    endcase
  end

  miriscv_cu control_unit (
    .clk_i              ( clk_i           ),
    .arstn_i            ( arstn_i         ),
    .branch_instr_i     ( d_branch        ),
    .branch_des_i       ( ex_branch       ),
    .jal_instr_i        ( d_jal           ),
    .jalr_instr_i       ( d_jalr          ),
    .illegal_instr_i    ( d_illegal_instr ),
    .rs1_data_i         ( rs1_data        ),
    .imm_i_i            ( imm_i           ),
    .imm_b_i            ( imm_b           ),
    .imm_j_i            ( imm_j           ),
    .cu_pc_addr_o       ( cu_pc_addr      ),
    .cu_instr_valid_o   ( cu_instr_valid  ),
    .mem_stall_req_i    ( mem_stall_req   ),
    .mem_kill_o         ( mem_kill        )
  );

  miriscv_rvfi_controller rvfi_ctrl(
    .clk_i            ( clk_i            ),
    .aresetn_i        ( arstn_i          ),

    .stall_req_i      ( mem_stall_req    ),
    .instr_valid_i    ( cu_instr_valid   ),
    .instr_i          ( fetched_instr    ),
    .illegal_instr_i  ( d_illegal_instr  ),
    .rs1_addr_i       ( rs1_addr         ),
    .rs2_addr_i       ( rs2_addr         ),
    .rs1_rdata_i      ( rs1_data         ),
    .rs2_rdata_i      ( rs2_data         ),
    .rd_addr_i        ( rd_addr          ),
    .rd_wdata_i       ( rd_data          ),
    .rd_we_i          ( rd_we            ),
    .pc_addr_i        ( cu_pc_addr       ),
    .dmem_rdata_i     ( data_rdata_i     ),
    .dmem_req_i       ( data_req_o       ),
    .dmem_we_i        ( data_we_o        ),
    .dmem_size_i      ( d_mem_size       ),
    .dmem_addr_i      ( data_addr_o      ),
    .dmem_wdata_i     ( data_wdata_o     ),
    .rvfi_valid_o     ( rvfi_valid_o     ),
    .rvfi_order_o     ( rvfi_order_o     ),
    .rvfi_insn_o      ( rvfi_insn_o      ),
    .rvfi_trap_o      ( rvfi_trap_o      ),
    .rvfi_halt_o      ( rvfi_halt_o      ),
    .rvfi_intr_o      ( rvfi_intr_o      ),
    .rvfi_mode_o      ( rvfi_mode_o      ),
    .rvfi_ixl_o       ( rvfi_ixl_o       ),
    .rvfi_rs1_addr_o  ( rvfi_rs1_addr_o  ),
    .rvfi_rs2_addr_o  ( rvfi_rs2_addr_o  ),
    .rvfi_rs1_rdata_o ( rvfi_rs1_rdata_o ),
    .rvfi_rs2_rdata_o ( rvfi_rs2_rdata_o ),
    .rvfi_rd_addr_o   ( rvfi_rd_addr_o   ),
    .rvfi_rd_wdata_o  ( rvfi_rd_wdata_o  ),
    .rvfi_pc_rdata_o  ( rvfi_pc_rdata_o  ),
    .rvfi_pc_wdata_o  ( rvfi_pc_wdata_o  ),
    .rvfi_mem_addr_o  ( rvfi_mem_addr_o  ),
    .rvfi_mem_rmask_o ( rvfi_mem_rmask_o ),
    .rvfi_mem_wmask_o ( rvfi_mem_wmask_o ),
    .rvfi_mem_rdata_o ( rvfi_mem_rdata_o ),
    .rvfi_mem_wdata_o ( rvfi_mem_wdata_o )
  );

endmodule
