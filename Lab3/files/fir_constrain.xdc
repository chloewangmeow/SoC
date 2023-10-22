create_clock -period 15.000 -name axis_clk -waveform {0.000 7.500} [get_ports axis_clk]

set _xlnx_shared_i0 [get_ports {{araddr[0]} {araddr[1]} {araddr[2]} {araddr[3]} {araddr[4]} {araddr[5]} {araddr[6]} {araddr[7]} {araddr[8]} {araddr[9]} {araddr[10]} {araddr[11]} arvalid {awaddr[0]} {awaddr[1]} {awaddr[2]} {awaddr[3]} {awaddr[4]} {awaddr[5]} {awaddr[6]} {awaddr[7]} {awaddr[8]} {awaddr[9]} {awaddr[10]} {awaddr[11]} awvalid axis_rst_n {data_Do[0]} {data_Do[1]} {data_Do[2]} {data_Do[3]} {data_Do[4]} {data_Do[5]} {data_Do[6]} {data_Do[7]} {data_Do[8]} {data_Do[9]} {data_Do[10]} {data_Do[11]} {data_Do[12]} {data_Do[13]} {data_Do[14]} {data_Do[15]} {data_Do[16]} {data_Do[17]} {data_Do[18]} {data_Do[19]} {data_Do[20]} {data_Do[21]} {data_Do[22]} {data_Do[23]} {data_Do[24]} {data_Do[25]} {data_Do[26]} {data_Do[27]} {data_Do[28]} {data_Do[29]} {data_Do[30]} {data_Do[31]} rready sm_tready {ss_tdata[0]} {ss_tdata[1]} {ss_tdata[2]} {ss_tdata[3]} {ss_tdata[4]} {ss_tdata[5]} {ss_tdata[6]} {ss_tdata[7]} {ss_tdata[8]} {ss_tdata[9]} {ss_tdata[10]} {ss_tdata[11]} {ss_tdata[12]} {ss_tdata[13]} {ss_tdata[14]} {ss_tdata[15]} {ss_tdata[16]} {ss_tdata[17]} {ss_tdata[18]} {ss_tdata[19]} {ss_tdata[20]} {ss_tdata[21]} {ss_tdata[22]} {ss_tdata[23]} {ss_tdata[24]} {ss_tdata[25]} {ss_tdata[26]} {ss_tdata[27]} {ss_tdata[28]} {ss_tdata[29]} {ss_tdata[30]} {ss_tdata[31]} ss_tlast ss_tvalid {tap_Do[0]} {tap_Do[1]} {tap_Do[2]} {tap_Do[3]} {tap_Do[4]} {tap_Do[5]} {tap_Do[6]} {tap_Do[7]} {tap_Do[8]} {tap_Do[9]} {tap_Do[10]} {tap_Do[11]} {tap_Do[12]} {tap_Do[13]} {tap_Do[14]} {tap_Do[15]} {tap_Do[16]} {tap_Do[17]} {tap_Do[18]} {tap_Do[19]} {tap_Do[20]} {tap_Do[21]} {tap_Do[22]} {tap_Do[23]} {tap_Do[24]} {tap_Do[25]} {tap_Do[26]} {tap_Do[27]} {tap_Do[28]} {tap_Do[29]} {tap_Do[30]} {tap_Do[31]} {wdata[0]} {wdata[1]} {wdata[2]} {wdata[3]} {wdata[4]} {wdata[5]} {wdata[6]} {wdata[7]} {wdata[8]} {wdata[9]} {wdata[10]} {wdata[11]} {wdata[12]} {wdata[13]} {wdata[14]} {wdata[15]} {wdata[16]} {wdata[17]} {wdata[18]} {wdata[19]} {wdata[20]} {wdata[21]} {wdata[22]} {wdata[23]} {wdata[24]} {wdata[25]} {wdata[26]} {wdata[27]} {wdata[28]} {wdata[29]} {wdata[30]} {wdata[31]} wvalid}]
set_input_delay -clock [get_clocks *clk*] 3.750 $_xlnx_shared_i0
set _xlnx_shared_i1 [get_ports -filter { NAME =~  "*" && DIRECTION == "OUT" }]
set_output_delay -clock [get_clocks *clk*] 3.750 $_xlnx_shared_i1














