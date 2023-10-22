module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11
)
(
    output  wire                     awready,
    output  wire                     wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
    output  wire                     arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  wire                     rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata,    
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  wire                     ss_tready, 
    input   wire                     sm_tready, 
    output  wire                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

reg r_awready, r_wready, r_arready, r_rvalid, r_ss_tready, r_sm_tvalid, r_sm_tlast;
reg [(pDATA_WIDTH-1):0] r_rdata, r_sm_tdata;
reg out_awready, out_wready, out_arready, out_rvalid, out_ss_tready, out_sm_tvalid, out_sm_tlast;
reg [(pDATA_WIDTH-1):0] out_rdata, out_sm_tdata;
reg m_tap_EN, m_data_EN;
reg [3:0] m_tap_WE, m_data_WE;
reg [(pDATA_WIDTH-1):0] m_tap_Di, m_data_Di;
reg [(pADDR_WIDTH-1):0] m_tap_A, m_data_A;


reg ap_start, n_ap_start, ap_done, n_ap_done, ap_idle, n_ap_idle;
reg out_rvalid_pre, out_rvalid_reg, last_flag, n_last_flag;
reg [10:0] data_num, n_data_num;
reg [(pDATA_WIDTH-1):0] y, pre_y, n_y;
reg [3:0] count_a, count_x, count, addr_flag, n_addr_flag;
reg [4:0] sum;
reg [1:0] state, n_state;
localparam IDLE = 2'd0, COEF = 2'd1, FIRR = 2'd2, IDLE2 = 2'd3;

// output
assign awready = r_awready;
assign wready = r_wready;
assign arready = r_arready;
assign rvalid = r_rvalid;
assign ss_tready = r_ss_tready;
assign sm_tvalid = r_sm_tvalid;
assign sm_tlast = r_sm_tlast;
assign rdata = r_rdata;
assign sm_tdata = r_sm_tdata;
assign tap_A = m_tap_A;
assign tap_Di = m_tap_Di;
assign tap_EN = m_tap_EN;
assign tap_WE = m_tap_WE;
assign data_A = m_data_A;
assign data_Di = m_data_Di;
assign data_EN = m_data_EN;
assign data_WE = m_data_WE;

// registers
always@(posedge axis_clk) begin
    if(~axis_rst_n) begin
        state <= IDLE;
        ap_start <= 1'b0;
        ap_done <= 1'b0;
        ap_idle <= 1'b1;
        data_num <= 10'b0;
        r_awready <= 1'b0;
        r_wready <= 1'b0;
        r_arready <= 1'b0;
        r_rvalid <= 1'b0;
        r_ss_tready <= 1'b0;
        r_sm_tvalid <= 1'b0;
        r_sm_tlast <= 1'b0;
        r_rdata <= 32'b0;
        r_sm_tdata <= 32'b0;
        out_rvalid_reg <= 1'b0;
        last_flag <= 1'b0;
        pre_y <= 32'b0;
        count_a <= 4'b0;
        count_x <= 4'b0;
        addr_flag <= 4'd1;
    end
    else begin
        state <= n_state;
        ap_start <= n_ap_start;
        ap_done <= n_ap_done;
        ap_idle <= n_ap_idle;
        data_num <= n_data_num;
        r_awready <= out_awready;
        r_wready <= out_wready;
        r_arready <= out_arready;
        r_rvalid <= out_rvalid;
        r_ss_tready <= out_ss_tready;
        r_sm_tvalid <= out_sm_tvalid;
        r_sm_tlast <= out_sm_tlast;
        r_rdata <= out_rdata;
        r_sm_tdata <= out_sm_tdata;
        out_rvalid_reg <= out_rvalid_pre;
        last_flag <= n_last_flag;
        pre_y <= n_y;
        count_a <= count;
        count_x <= count_a;
        addr_flag <= n_addr_flag;
    end
end

// read address
always@* begin
    if(r_rvalid) begin
        out_rvalid = 1'b0;
        out_rdata = 32'b0;
    end
    else if(state == COEF & araddr[7] & arvalid) begin
        out_rdata = tap_Do;
        out_rvalid = out_rvalid_reg;
    end
    else if (~araddr[7] & ~araddr[4] & arvalid) begin
        out_rdata = {29'b0, ap_idle, ap_done, ap_start};
        out_rvalid = 1'b1;
    end
    else begin
        out_rdata = 32'b0;
        out_rvalid = 1'b0;
    end 

    if(state == COEF & araddr[7] & arvalid) begin
        out_rvalid_pre = 1'b1;
    end
    else begin
        out_rvalid_pre = 1'b0;
    end
end

// ap_start
always@* begin
    if(~ap_idle) begin
        n_ap_start = 0;
    end
    else if(~awaddr[7] & ~awaddr[4] & awvalid & wvalid) begin
        n_ap_start = wdata[0];
    end
    else begin
        n_ap_start = ap_start;
    end
end

// ap_idle
always@* begin
    if(n_ap_start) begin
        n_ap_idle = 0;
    end
    else if(state == IDLE2 & ap_done & r_rvalid) begin
        n_ap_idle = 1;
    end
    else begin
        n_ap_idle = ap_idle;
    end
end

// ap_done
always@* begin
    if(out_sm_tlast) begin
        n_ap_done = 1'b1;
    end
    else if(ap_idle) begin
        n_ap_done = 1'b0;
    end
    else begin
        n_ap_done = ap_done;
    end
end

// FSM
always@* begin
    case(state)
    IDLE: begin
        n_state = COEF;
    end
    COEF: begin
        if(ap_start == 1) begin
            n_state = FIRR;
        end
        else begin
            n_state = COEF;
        end
    end
    FIRR: begin
        if(ap_done == 1) begin
            n_state = IDLE2;
        end
        else begin
            n_state = FIRR;
        end
    end
    IDLE2: begin
        if(ap_start == 1) begin
            n_state = FIRR;
        end
        else begin
            n_state = IDLE2;
        end
    end
    default: begin
        n_state = IDLE;
    end
    endcase
end

// data_num
always@* begin
    if(~awaddr[7] & awaddr[4] & awvalid & wvalid) begin
        n_data_num = wdata[11:0];
    end
    else begin
        n_data_num = data_num;
    end
end

// coef address
always@* begin
    if(state == COEF & awvalid & awaddr[7]) begin
        m_tap_A = {5'b0, awaddr[6:0]};
        m_tap_EN = r_wready;
        m_tap_WE = {4{r_wready}};
        m_tap_Di = wdata;
    end
    else if(state == COEF & arvalid & araddr[7]) begin
        m_tap_A = {5'b0, araddr[6:0]};
        m_tap_EN = out_rvalid;
        m_tap_WE = 4'b0;
        m_tap_Di = 32'b0;
    end
    else if(state == FIRR) begin
        m_tap_A = {6'b0, count_a, 2'b0};
        m_tap_EN = 1'b1;
        m_tap_WE = 4'b0;
        m_tap_Di = 32'b0;
    end
    else begin
        m_tap_A = 12'b0;
        m_tap_EN = 1'b0;
        m_tap_WE = 4'b0;
        m_tap_Di = 32'b0;
    end
end

// address flag
always@* begin
    if((count_a == 4'd10) & (addr_flag == 4'd10)) begin
        n_addr_flag = 4'd0;
    end
    else if(count_a == 4'd10) begin
        n_addr_flag = addr_flag + 1'b1;
    end
    else begin
        n_addr_flag = addr_flag;
    end
end

// data addr
always@* begin
    sum = count_a + addr_flag;
    m_data_WE = (state == COEF) ? m_tap_WE : (count_a == 4'd10) ? {4{1'b1}} : 4'b0;
    m_data_A  = (state == COEF) ? m_tap_A  : (sum <= 5'd10) ? {5'b0, sum, 2'b0} : {5'b0, (sum - 5'd11), 2'b0};
    m_data_Di = (state == COEF) ? 32'b0    : ss_tdata;
    m_data_EN = (state == FIRR) | m_tap_WE[0];
end

// control
always@* begin
    n_last_flag = (ss_tlast == 1'b1) ? 1'b1 : ((out_sm_tlast == 1'b1) ? 1'b0 : last_flag) ;
    out_ss_tready = (count == 4'd10);
    out_sm_tdata = y;
    out_sm_tvalid = (count_x == 4'd10);
    out_sm_tlast = ((last_flag == 1'b1) & (count_x == 4'd10));
    out_arready = 1;//out_rvalid;
    out_awready = 1;//awvalid; //(r_awready == 1'b1) ? 1'b0 : awvalid;
    out_wready = 1;//out_awready;
end 

// counter
always@* begin
    if ((count_a == 4'd10) | (state != FIRR)) begin
        count = 0;
    end
    else begin
        count = count_a + 1'b1;
    end
end

// FIR
always@* begin
    y = (tap_Do * data_Do) + pre_y;
    if (state != FIRR | (count_a == 4'd0)) begin
        n_y = 32'd0;
    end
    else begin
        n_y = y;
    end
end

endmodule