# Definitional proc to organize widgets for parameters.
proc init_gui { IPINST } {
  ipgui::add_param $IPINST -name "Component_Name"
  #Adding Page
  set Page_0 [ipgui::add_page $IPINST -name "Page 0"]
  ipgui::add_param $IPINST -name "AXI_DATA_WIDTH" -parent ${Page_0}
  ipgui::add_param $IPINST -name "CTRL_WRITE_ENABLE_POSEDGE_TRIGGERED" -parent ${Page_0}
  ipgui::add_param $IPINST -name "XLEN" -parent ${Page_0}


}

proc update_PARAM_VALUE.AXI_DATA_WIDTH { PARAM_VALUE.AXI_DATA_WIDTH } {
	# Procedure called to update AXI_DATA_WIDTH when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.AXI_DATA_WIDTH { PARAM_VALUE.AXI_DATA_WIDTH } {
	# Procedure called to validate AXI_DATA_WIDTH
	return true
}

proc update_PARAM_VALUE.CTRL_WRITE_ENABLE_POSEDGE_TRIGGERED { PARAM_VALUE.CTRL_WRITE_ENABLE_POSEDGE_TRIGGERED } {
	# Procedure called to update CTRL_WRITE_ENABLE_POSEDGE_TRIGGERED when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.CTRL_WRITE_ENABLE_POSEDGE_TRIGGERED { PARAM_VALUE.CTRL_WRITE_ENABLE_POSEDGE_TRIGGERED } {
	# Procedure called to validate CTRL_WRITE_ENABLE_POSEDGE_TRIGGERED
	return true
}

proc update_PARAM_VALUE.XLEN { PARAM_VALUE.XLEN } {
	# Procedure called to update XLEN when any of the dependent parameters in the arguments change
}

proc validate_PARAM_VALUE.XLEN { PARAM_VALUE.XLEN } {
	# Procedure called to validate XLEN
	return true
}


proc update_MODELPARAM_VALUE.XLEN { MODELPARAM_VALUE.XLEN PARAM_VALUE.XLEN } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.XLEN}] ${MODELPARAM_VALUE.XLEN}
}

proc update_MODELPARAM_VALUE.AXI_DATA_WIDTH { MODELPARAM_VALUE.AXI_DATA_WIDTH PARAM_VALUE.AXI_DATA_WIDTH } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.AXI_DATA_WIDTH}] ${MODELPARAM_VALUE.AXI_DATA_WIDTH}
}

proc update_MODELPARAM_VALUE.CTRL_WRITE_ENABLE_POSEDGE_TRIGGERED { MODELPARAM_VALUE.CTRL_WRITE_ENABLE_POSEDGE_TRIGGERED PARAM_VALUE.CTRL_WRITE_ENABLE_POSEDGE_TRIGGERED } {
	# Procedure called to set VHDL generic/Verilog parameter value(s) based on TCL parameter value
	set_property value [get_property value ${PARAM_VALUE.CTRL_WRITE_ENABLE_POSEDGE_TRIGGERED}] ${MODELPARAM_VALUE.CTRL_WRITE_ENABLE_POSEDGE_TRIGGERED}
}

