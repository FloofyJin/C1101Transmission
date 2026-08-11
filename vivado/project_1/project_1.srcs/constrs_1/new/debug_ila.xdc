create_debug_core u_ila_0 ila
set_property ALL_PROBE_SAME_MU true [get_debug_cores u_ila_0]
set_property ALL_PROBE_SAME_MU_CNT 1 [get_debug_cores u_ila_0]
set_property C_ADV_TRIGGER false [get_debug_cores u_ila_0]
set_property C_DATA_DEPTH 4096 [get_debug_cores u_ila_0]
set_property C_EN_STRG_QUAL false [get_debug_cores u_ila_0]
set_property C_INPUT_PIPE_STAGES 0 [get_debug_cores u_ila_0]
set_property C_TRIGIN_EN false [get_debug_cores u_ila_0]
set_property C_TRIGOUT_EN false [get_debug_cores u_ila_0]
set_property port_width 1 [get_debug_ports u_ila_0/clk]
connect_debug_port u_ila_0/clk [get_nets [list sysclk_IBUF_BUFG]]
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe0]
set_property port_width 8 [get_debug_ports u_ila_0/probe0]
connect_debug_port u_ila_0/probe0 [get_nets [list {rf_i/b_rssi[0]} {rf_i/b_rssi[1]} {rf_i/b_rssi[2]} {rf_i/b_rssi[3]} {rf_i/b_rssi[4]} {rf_i/b_rssi[5]} {rf_i/b_rssi[6]} {rf_i/b_rssi[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe1]
set_property port_width 3 [get_debug_ports u_ila_0/probe1]
connect_debug_port u_ila_0/probe1 [get_nets [list {rf_i/cmd[0]} {rf_i/cmd[1]} {rf_i/cmd[2]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe2]
set_property port_width 8 [get_debug_ports u_ila_0/probe2]
connect_debug_port u_ila_0/probe2 [get_nets [list {rf_i/b_fail_addr[0]} {rf_i/b_fail_addr[1]} {rf_i/b_fail_addr[2]} {rf_i/b_fail_addr[3]} {rf_i/b_fail_addr[4]} {rf_i/b_fail_addr[5]} {rf_i/b_fail_addr[6]} {rf_i/b_fail_addr[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe3]
set_property port_width 3 [get_debug_ports u_ila_0/probe3]
connect_debug_port u_ila_0/probe3 [get_nets [list {rf_i/b_cmd[0]} {rf_i/b_cmd[1]} {rf_i/b_cmd[2]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe4]
set_property port_width 8 [get_debug_ports u_ila_0/probe4]
connect_debug_port u_ila_0/probe4 [get_nets [list {rf_i/b_count[0]} {rf_i/b_count[1]} {rf_i/b_count[2]} {rf_i/b_count[3]} {rf_i/b_count[4]} {rf_i/b_count[5]} {rf_i/b_count[6]} {rf_i/b_count[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe5]
set_property port_width 8 [get_debug_ports u_ila_0/probe5]
connect_debug_port u_ila_0/probe5 [get_nets [list {rf_i/b_fail_value[0]} {rf_i/b_fail_value[1]} {rf_i/b_fail_value[2]} {rf_i/b_fail_value[3]} {rf_i/b_fail_value[4]} {rf_i/b_fail_value[5]} {rf_i/b_fail_value[6]} {rf_i/b_fail_value[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe6]
set_property port_width 8 [get_debug_ports u_ila_0/probe6]
connect_debug_port u_ila_0/probe6 [get_nets [list {rf_i/b_rxbytes[0]} {rf_i/b_rxbytes[1]} {rf_i/b_rxbytes[2]} {rf_i/b_rxbytes[3]} {rf_i/b_rxbytes[4]} {rf_i/b_rxbytes[5]} {rf_i/b_rxbytes[6]} {rf_i/b_rxbytes[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe7]
set_property port_width 16 [get_debug_ports u_ila_0/probe7]
connect_debug_port u_ila_0/probe7 [get_nets [list {rf_i/b_rx_count[0]} {rf_i/b_rx_count[1]} {rf_i/b_rx_count[2]} {rf_i/b_rx_count[3]} {rf_i/b_rx_count[4]} {rf_i/b_rx_count[5]} {rf_i/b_rx_count[6]} {rf_i/b_rx_count[7]} {rf_i/b_rx_count[8]} {rf_i/b_rx_count[9]} {rf_i/b_rx_count[10]} {rf_i/b_rx_count[11]} {rf_i/b_rx_count[12]} {rf_i/b_rx_count[13]} {rf_i/b_rx_count[14]} {rf_i/b_rx_count[15]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe8]
set_property port_width 8 [get_debug_ports u_ila_0/probe8]
connect_debug_port u_ila_0/probe8 [get_nets [list {rf_i/b_spi_tx[0]} {rf_i/b_spi_tx[1]} {rf_i/b_spi_tx[2]} {rf_i/b_spi_tx[3]} {rf_i/b_spi_tx[4]} {rf_i/b_spi_tx[5]} {rf_i/b_spi_tx[6]} {rf_i/b_spi_tx[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe9]
set_property port_width 8 [get_debug_ports u_ila_0/probe9]
connect_debug_port u_ila_0/probe9 [get_nets [list {rf_i/b_start_index[0]} {rf_i/b_start_index[1]} {rf_i/b_start_index[2]} {rf_i/b_start_index[3]} {rf_i/b_start_index[4]} {rf_i/b_start_index[5]} {rf_i/b_start_index[6]} {rf_i/b_start_index[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe10]
set_property port_width 16 [get_debug_ports u_ila_0/probe10]
connect_debug_port u_ila_0/probe10 [get_nets [list {rf_i/a_pkt_count[0]} {rf_i/a_pkt_count[1]} {rf_i/a_pkt_count[2]} {rf_i/a_pkt_count[3]} {rf_i/a_pkt_count[4]} {rf_i/a_pkt_count[5]} {rf_i/a_pkt_count[6]} {rf_i/a_pkt_count[7]} {rf_i/a_pkt_count[8]} {rf_i/a_pkt_count[9]} {rf_i/a_pkt_count[10]} {rf_i/a_pkt_count[11]} {rf_i/a_pkt_count[12]} {rf_i/a_pkt_count[13]} {rf_i/a_pkt_count[14]} {rf_i/a_pkt_count[15]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe11]
set_property port_width 8 [get_debug_ports u_ila_0/probe11]
connect_debug_port u_ila_0/probe11 [get_nets [list {rf_i/a_fail_value[0]} {rf_i/a_fail_value[1]} {rf_i/a_fail_value[2]} {rf_i/a_fail_value[3]} {rf_i/a_fail_value[4]} {rf_i/a_fail_value[5]} {rf_i/a_fail_value[6]} {rf_i/a_fail_value[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe12]
set_property port_width 8 [get_debug_ports u_ila_0/probe12]
connect_debug_port u_ila_0/probe12 [get_nets [list {rf_i/cmd_addr[0]} {rf_i/cmd_addr[1]} {rf_i/cmd_addr[2]} {rf_i/cmd_addr[3]} {rf_i/cmd_addr[4]} {rf_i/cmd_addr[5]} {rf_i/cmd_addr[6]} {rf_i/cmd_addr[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe13]
set_property port_width 8 [get_debug_ports u_ila_0/probe13]
connect_debug_port u_ila_0/probe13 [get_nets [list {rf_i/a_fail_addr[0]} {rf_i/a_fail_addr[1]} {rf_i/a_fail_addr[2]} {rf_i/a_fail_addr[3]} {rf_i/a_fail_addr[4]} {rf_i/a_fail_addr[5]} {rf_i/a_fail_addr[6]} {rf_i/a_fail_addr[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe14]
set_property port_width 16 [get_debug_ports u_ila_0/probe14]
connect_debug_port u_ila_0/probe14 [get_nets [list {rf_i/b_pt_writes[0]} {rf_i/b_pt_writes[1]} {rf_i/b_pt_writes[2]} {rf_i/b_pt_writes[3]} {rf_i/b_pt_writes[4]} {rf_i/b_pt_writes[5]} {rf_i/b_pt_writes[6]} {rf_i/b_pt_writes[7]} {rf_i/b_pt_writes[8]} {rf_i/b_pt_writes[9]} {rf_i/b_pt_writes[10]} {rf_i/b_pt_writes[11]} {rf_i/b_pt_writes[12]} {rf_i/b_pt_writes[13]} {rf_i/b_pt_writes[14]} {rf_i/b_pt_writes[15]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe15]
set_property port_width 12 [get_debug_ports u_ila_0/probe15]
connect_debug_port u_ila_0/probe15 [get_nets [list {rf_i/draw_y[0]} {rf_i/draw_y[1]} {rf_i/draw_y[2]} {rf_i/draw_y[3]} {rf_i/draw_y[4]} {rf_i/draw_y[5]} {rf_i/draw_y[6]} {rf_i/draw_y[7]} {rf_i/draw_y[8]} {rf_i/draw_y[9]} {rf_i/draw_y[10]} {rf_i/draw_y[11]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe16]
set_property port_width 8 [get_debug_ports u_ila_0/probe16]
connect_debug_port u_ila_0/probe16 [get_nets [list {rf_i/a_marcstate[0]} {rf_i/a_marcstate[1]} {rf_i/a_marcstate[2]} {rf_i/a_marcstate[3]} {rf_i/a_marcstate[4]} {rf_i/a_marcstate[5]} {rf_i/a_marcstate[6]} {rf_i/a_marcstate[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe17]
set_property port_width 8 [get_debug_ports u_ila_0/probe17]
connect_debug_port u_ila_0/probe17 [get_nets [list {rf_i/b_rd_data[0]} {rf_i/b_rd_data[1]} {rf_i/b_rd_data[2]} {rf_i/b_rd_data[3]} {rf_i/b_rd_data[4]} {rf_i/b_rd_data[5]} {rf_i/b_rd_data[6]} {rf_i/b_rd_data[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe18]
set_property port_width 8 [get_debug_ports u_ila_0/probe18]
connect_debug_port u_ila_0/probe18 [get_nets [list {rf_i/pt_addr[0]} {rf_i/pt_addr[1]} {rf_i/pt_addr[2]} {rf_i/pt_addr[3]} {rf_i/pt_addr[4]} {rf_i/pt_addr[5]} {rf_i/pt_addr[6]} {rf_i/pt_addr[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe19]
set_property port_width 18 [get_debug_ports u_ila_0/probe19]
connect_debug_port u_ila_0/probe19 [get_nets [list {rf_i/pt_data[0]} {rf_i/pt_data[1]} {rf_i/pt_data[2]} {rf_i/pt_data[3]} {rf_i/pt_data[4]} {rf_i/pt_data[5]} {rf_i/pt_data[6]} {rf_i/pt_data[7]} {rf_i/pt_data[8]} {rf_i/pt_data[9]} {rf_i/pt_data[10]} {rf_i/pt_data[11]} {rf_i/pt_data[12]} {rf_i/pt_data[13]} {rf_i/pt_data[14]} {rf_i/pt_data[15]} {rf_i/pt_data[16]} {rf_i/pt_data[17]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe20]
set_property port_width 8 [get_debug_ports u_ila_0/probe20]
connect_debug_port u_ila_0/probe20 [get_nets [list {rf_i/pt_raddr[0]} {rf_i/pt_raddr[1]} {rf_i/pt_raddr[2]} {rf_i/pt_raddr[3]} {rf_i/pt_raddr[4]} {rf_i/pt_raddr[5]} {rf_i/pt_raddr[6]} {rf_i/pt_raddr[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe21]
set_property port_width 7 [get_debug_ports u_ila_0/probe21]
connect_debug_port u_ila_0/probe21 [get_nets [list {rf_i/b_lqi[0]} {rf_i/b_lqi[1]} {rf_i/b_lqi[2]} {rf_i/b_lqi[3]} {rf_i/b_lqi[4]} {rf_i/b_lqi[5]} {rf_i/b_lqi[6]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe22]
set_property port_width 8 [get_debug_ports u_ila_0/probe22]
connect_debug_port u_ila_0/probe22 [get_nets [list {rf_i/b_cmd_addr[0]} {rf_i/b_cmd_addr[1]} {rf_i/b_cmd_addr[2]} {rf_i/b_cmd_addr[3]} {rf_i/b_cmd_addr[4]} {rf_i/b_cmd_addr[5]} {rf_i/b_cmd_addr[6]} {rf_i/b_cmd_addr[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe23]
set_property port_width 18 [get_debug_ports u_ila_0/probe23]
connect_debug_port u_ila_0/probe23 [get_nets [list {rf_i/pt_rdata[0]} {rf_i/pt_rdata[1]} {rf_i/pt_rdata[2]} {rf_i/pt_rdata[3]} {rf_i/pt_rdata[4]} {rf_i/pt_rdata[5]} {rf_i/pt_rdata[6]} {rf_i/pt_rdata[7]} {rf_i/pt_rdata[8]} {rf_i/pt_rdata[9]} {rf_i/pt_rdata[10]} {rf_i/pt_rdata[11]} {rf_i/pt_rdata[12]} {rf_i/pt_rdata[13]} {rf_i/pt_rdata[14]} {rf_i/pt_rdata[15]} {rf_i/pt_rdata[16]} {rf_i/pt_rdata[17]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe24]
set_property port_width 8 [get_debug_ports u_ila_0/probe24]
connect_debug_port u_ila_0/probe24 [get_nets [list {rf_i/rd_data[0]} {rf_i/rd_data[1]} {rf_i/rd_data[2]} {rf_i/rd_data[3]} {rf_i/rd_data[4]} {rf_i/rd_data[5]} {rf_i/rd_data[6]} {rf_i/rd_data[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe25]
set_property port_width 8 [get_debug_ports u_ila_0/probe25]
connect_debug_port u_ila_0/probe25 [get_nets [list {rf_i/sc_corner[0]} {rf_i/sc_corner[1]} {rf_i/sc_corner[2]} {rf_i/sc_corner[3]} {rf_i/sc_corner[4]} {rf_i/sc_corner[5]} {rf_i/sc_corner[6]} {rf_i/sc_corner[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe26]
set_property port_width 8 [get_debug_ports u_ila_0/probe26]
connect_debug_port u_ila_0/probe26 [get_nets [list {rf_i/spi_rx[0]} {rf_i/spi_rx[1]} {rf_i/spi_rx[2]} {rf_i/spi_rx[3]} {rf_i/spi_rx[4]} {rf_i/spi_rx[5]} {rf_i/spi_rx[6]} {rf_i/spi_rx[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe27]
set_property port_width 8 [get_debug_ports u_ila_0/probe27]
connect_debug_port u_ila_0/probe27 [get_nets [list {rf_i/spi_tx[0]} {rf_i/spi_tx[1]} {rf_i/spi_tx[2]} {rf_i/spi_tx[3]} {rf_i/spi_tx[4]} {rf_i/spi_tx[5]} {rf_i/spi_tx[6]} {rf_i/spi_tx[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe28]
set_property port_width 8 [get_debug_ports u_ila_0/probe28]
connect_debug_port u_ila_0/probe28 [get_nets [list {rf_i/b_len_byte[0]} {rf_i/b_len_byte[1]} {rf_i/b_len_byte[2]} {rf_i/b_len_byte[3]} {rf_i/b_len_byte[4]} {rf_i/b_len_byte[5]} {rf_i/b_len_byte[6]} {rf_i/b_len_byte[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe29]
set_property port_width 8 [get_debug_ports u_ila_0/probe29]
connect_debug_port u_ila_0/probe29 [get_nets [list {rf_i/a_txbytes[0]} {rf_i/a_txbytes[1]} {rf_i/a_txbytes[2]} {rf_i/a_txbytes[3]} {rf_i/a_txbytes[4]} {rf_i/a_txbytes[5]} {rf_i/a_txbytes[6]} {rf_i/a_txbytes[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe30]
set_property port_width 12 [get_debug_ports u_ila_0/probe30]
connect_debug_port u_ila_0/probe30 [get_nets [list {rf_i/draw_x[0]} {rf_i/draw_x[1]} {rf_i/draw_x[2]} {rf_i/draw_x[3]} {rf_i/draw_x[4]} {rf_i/draw_x[5]} {rf_i/draw_x[6]} {rf_i/draw_x[7]} {rf_i/draw_x[8]} {rf_i/draw_x[9]} {rf_i/draw_x[10]} {rf_i/draw_x[11]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe31]
set_property port_width 8 [get_debug_ports u_ila_0/probe31]
connect_debug_port u_ila_0/probe31 [get_nets [list {rf_i/b_spi_rx[0]} {rf_i/b_spi_rx[1]} {rf_i/b_spi_rx[2]} {rf_i/b_spi_rx[3]} {rf_i/b_spi_rx[4]} {rf_i/b_spi_rx[5]} {rf_i/b_spi_rx[6]} {rf_i/b_spi_rx[7]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe32]
set_property port_width 16 [get_debug_ports u_ila_0/probe32]
connect_debug_port u_ila_0/probe32 [get_nets [list {rf_i/b_err_count[0]} {rf_i/b_err_count[1]} {rf_i/b_err_count[2]} {rf_i/b_err_count[3]} {rf_i/b_err_count[4]} {rf_i/b_err_count[5]} {rf_i/b_err_count[6]} {rf_i/b_err_count[7]} {rf_i/b_err_count[8]} {rf_i/b_err_count[9]} {rf_i/b_err_count[10]} {rf_i/b_err_count[11]} {rf_i/b_err_count[12]} {rf_i/b_err_count[13]} {rf_i/b_err_count[14]} {rf_i/b_err_count[15]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe33]
set_property port_width 16 [get_debug_ports u_ila_0/probe33]
connect_debug_port u_ila_0/probe33 [get_nets [list {rf_i/sc_frames[0]} {rf_i/sc_frames[1]} {rf_i/sc_frames[2]} {rf_i/sc_frames[3]} {rf_i/sc_frames[4]} {rf_i/sc_frames[5]} {rf_i/sc_frames[6]} {rf_i/sc_frames[7]} {rf_i/sc_frames[8]} {rf_i/sc_frames[9]} {rf_i/sc_frames[10]} {rf_i/sc_frames[11]} {rf_i/sc_frames[12]} {rf_i/sc_frames[13]} {rf_i/sc_frames[14]} {rf_i/sc_frames[15]}]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe34]
set_property port_width 1 [get_debug_ports u_ila_0/probe34]
connect_debug_port u_ila_0/probe34 [get_nets [list rf_i/a_cfg_busy]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe35]
set_property port_width 1 [get_debug_ports u_ila_0/probe35]
connect_debug_port u_ila_0/probe35 [get_nets [list rf_i/a_cfg_done]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe36]
set_property port_width 1 [get_debug_ports u_ila_0/probe36]
connect_debug_port u_ila_0/probe36 [get_nets [list rf_i/a_cfg_timeout]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe37]
set_property port_width 1 [get_debug_ports u_ila_0/probe37]
connect_debug_port u_ila_0/probe37 [get_nets [list rf_i/a_config_ok]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe38]
set_property port_width 1 [get_debug_ports u_ila_0/probe38]
connect_debug_port u_ila_0/probe38 [get_nets [list rf_i/a_sent_ok]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe39]
set_property port_width 1 [get_debug_ports u_ila_0/probe39]
connect_debug_port u_ila_0/probe39 [get_nets [list rf_i/a_sync_seen]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe40]
set_property port_width 1 [get_debug_ports u_ila_0/probe40]
connect_debug_port u_ila_0/probe40 [get_nets [list rf_i/a_timeout]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe41]
set_property port_width 1 [get_debug_ports u_ila_0/probe41]
connect_debug_port u_ila_0/probe41 [get_nets [list rf_i/a_tx_busy]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe42]
set_property port_width 1 [get_debug_ports u_ila_0/probe42]
connect_debug_port u_ila_0/probe42 [get_nets [list rf_i/a_tx_done]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe43]
set_property port_width 1 [get_debug_ports u_ila_0/probe43]
connect_debug_port u_ila_0/probe43 [get_nets [list rf_i/b_bad_fmt]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe44]
set_property port_width 1 [get_debug_ports u_ila_0/probe44]
connect_debug_port u_ila_0/probe44 [get_nets [list rf_i/b_cfg_busy]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe45]
set_property port_width 1 [get_debug_ports u_ila_0/probe45]
connect_debug_port u_ila_0/probe45 [get_nets [list rf_i/b_cfg_done]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe46]
set_property port_width 1 [get_debug_ports u_ila_0/probe46]
connect_debug_port u_ila_0/probe46 [get_nets [list rf_i/b_cfg_timeout]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe47]
set_property port_width 1 [get_debug_ports u_ila_0/probe47]
connect_debug_port u_ila_0/probe47 [get_nets [list rf_i/b_config_ok]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe48]
set_property port_width 1 [get_debug_ports u_ila_0/probe48]
connect_debug_port u_ila_0/probe48 [get_nets [list rf_i/b_crc_ok]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe49]
set_property port_width 1 [get_debug_ports u_ila_0/probe49]
connect_debug_port u_ila_0/probe49 [get_nets [list rf_i/b_csn_dbg]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe50]
set_property port_width 1 [get_debug_ports u_ila_0/probe50]
connect_debug_port u_ila_0/probe50 [get_nets [list rf_i/b_fmt_ok]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe51]
set_property port_width 1 [get_debug_ports u_ila_0/probe51]
connect_debug_port u_ila_0/probe51 [get_nets [list rf_i/b_gdo2_s]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe52]
set_property port_width 1 [get_debug_ports u_ila_0/probe52]
connect_debug_port u_ila_0/probe52 [get_nets [list rf_i/b_miso_s]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe53]
set_property port_width 1 [get_debug_ports u_ila_0/probe53]
connect_debug_port u_ila_0/probe53 [get_nets [list rf_i/b_overflow]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe54]
set_property port_width 1 [get_debug_ports u_ila_0/probe54]
connect_debug_port u_ila_0/probe54 [get_nets [list rf_i/b_pkt_ok]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe55]
set_property port_width 1 [get_debug_ports u_ila_0/probe55]
connect_debug_port u_ila_0/probe55 [get_nets [list rf_i/b_rx_busy]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe56]
set_property port_width 1 [get_debug_ports u_ila_0/probe56]
connect_debug_port u_ila_0/probe56 [get_nets [list rf_i/b_rx_done]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe57]
set_property port_width 1 [get_debug_ports u_ila_0/probe57]
connect_debug_port u_ila_0/probe57 [get_nets [list rf_i/b_rx_timeout]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe58]
set_property port_width 1 [get_debug_ports u_ila_0/probe58]
connect_debug_port u_ila_0/probe58 [get_nets [list rf_i/b_spi_busy]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe59]
set_property port_width 1 [get_debug_ports u_ila_0/probe59]
connect_debug_port u_ila_0/probe59 [get_nets [list rf_i/b_spi_done]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe60]
set_property port_width 1 [get_debug_ports u_ila_0/probe60]
connect_debug_port u_ila_0/probe60 [get_nets [list rf_i/b_spi_hold]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe61]
set_property port_width 1 [get_debug_ports u_ila_0/probe61]
connect_debug_port u_ila_0/probe61 [get_nets [list rf_i/b_spi_start]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe62]
set_property port_width 1 [get_debug_ports u_ila_0/probe62]
connect_debug_port u_ila_0/probe62 [get_nets [list rf_i/csn_dbg]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe63]
set_property port_width 1 [get_debug_ports u_ila_0/probe63]
connect_debug_port u_ila_0/probe63 [get_nets [list rf_i/gdo2_s]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe64]
set_property port_width 1 [get_debug_ports u_ila_0/probe64]
connect_debug_port u_ila_0/probe64 [get_nets [list rf_i/miso_s]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe65]
set_property port_width 1 [get_debug_ports u_ila_0/probe65]
connect_debug_port u_ila_0/probe65 [get_nets [list rf_i/pt_we]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe66]
set_property port_width 1 [get_debug_ports u_ila_0/probe66]
connect_debug_port u_ila_0/probe66 [get_nets [list rf_i/spi_busy]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe67]
set_property port_width 1 [get_debug_ports u_ila_0/probe67]
connect_debug_port u_ila_0/probe67 [get_nets [list rf_i/spi_done]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe68]
set_property port_width 1 [get_debug_ports u_ila_0/probe68]
connect_debug_port u_ila_0/probe68 [get_nets [list rf_i/spi_hold]]
create_debug_port u_ila_0 probe
set_property PROBE_TYPE DATA_AND_TRIGGER [get_debug_ports u_ila_0/probe69]
set_property port_width 1 [get_debug_ports u_ila_0/probe69]
connect_debug_port u_ila_0/probe69 [get_nets [list rf_i/spi_start]]
set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets sysclk_IBUF_BUFG]
