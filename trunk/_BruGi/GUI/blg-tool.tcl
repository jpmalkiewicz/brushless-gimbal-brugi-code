#!/usr/bin/wish
#
# Copyright by Oliver Dippel <oliver@multixmedia.org>
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# any later version. see <http://www.gnu.org/licenses/>
#
# enhancements by Alois Hahn
#
package require Tk

set VERSION "2014-01-10 / for BruGi Firmware v50 r199 or higher"

# just activate a debug console
#catch {console show}

#####################################################################################
# Big hexdata
#####################################################################################

set HEXDATA ""

image create photo logo -data {
R0lGODlhyAA2APcAAFxbV1tbW3RsX3hsX2NjY25uZ25sbH9nZ3NsZnpuZXJra3tsanlxZXZzbXty
a21tc21sfHFrc3BsfmxxdWxze3Nzc3t1cnR6d314cXN0fHV5e3x7e4JtZoNtaY5sbod2Z4RzbYlx
bod7aIt8bZB4bo1vcYlxc4R5cYp+dIN+e5V+dZR+eo6FdI6EeZKEdZyBd5eIcZGFepWKfp2Me6yP
c6qQc6SSfmxtgXJqgW91g2x9hG16iHR2gnZ7hHp9g3V6inl9i3yBhneBiHyDjH6Ij3qIkHmGmn6M
mYSEhYiFgoSDi4GMj42NjZeHgpWMhJyMhJCLip2ThZSRi5yUiJ+ejIKHlYmHl4SJkoiNkISGmoOM
m4qJm4WRnoyUnJWVlZuYk5OXnJubnKGUg66TgKSYh62ag6Wai62cjaWblKedma+gj7Clj66ikayl
ma+pmbWik7qlkreqlbqplbWlm7qnmLapm7iqnrGwmIKNoIiNpIaSpIiQoo6Zp4SQqJKUo5Sdq5Kf
spOkrZqhrZ2msJurtpumupqsu52yvamloKGkq6Wqr6mrrLiqoryxorSzrb2+rqWqsKqtsqOtvKqv
v66xtaKwurKztrO1ubW5vL29v8Ctnca1mMKxpMq5psCzqcO9rMy9rc2+sdC/stTEr9LEs9jHsdvI
tNXMut7PuOHUvKevwqS1xqy2xKe5wqu6xaOzy6e/zKq7yrC8xru+wq7Cz77AxavI1rPF07DI0r3P
3b7R1cLExcLEyc3Nz9HR0ePWxOjXxOPaxOnbxM7i3+zgxOvizfDkzu7l0PHo1s7j6tHj5dHm69Ho
6/385f776eP0+gAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAACH5BAEAANAALAAAAADIADYA
AAj+AKEJHEiwoMGDCBMqXMiwocOHECNKnEixosWLGDNq3Mixo8ePIEOKHEmypMmTKFOqXMmypcuX
MGPKnEmzps2bOHPq3Mmzp8+fQIMKHUq0qNGjSJMqXcq0qdOnODNBW7TBAAECBgwEgcp1YyZLvbwE
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
}


image create photo sensor -data {
R0lGODlheABmAOf8ACQlJiUmKScoKysrLSgpJy4wLzExLi0uMjEvMy8wMy4yOTIzNTY4Nzk5NjU2
OTc4Ozo7PTk3OjEvLBwdH0c9PFg9OFI6L3I6Mz5BPz5GN0FBPklFOlhEOlVRPWRIO3RLOz0+QTg7
Q0I+REk+ST9BQz1IRzlVVEJDRUtEREVJRkpKRkVFSUpFS0ZJTEpLTVNFQ1JNTFhKR05RTkdSSlJS
TVlTSkZMUkxNUkpJVlJNU1ZMVk5RU0xTW0hVV1NTVFpUVFZYVlpZVlRVWVtUW1ZZW1tbXGVKR2dT
SXdVSmNcW2hYV3ZbWHJNRl1hXlVlXEZgX2JiXmpjXXhjW21gTktaYl1eYVZZY2JdZGpeZ3Jeal5h
Y11laltoaFZsbmNjZGtkZGVpZmtqZmVlaWpla2Vpa2trbHJsbHdoZ25xbmVxbHJybnlybWVtcmxt
cmhpdHJtdHdqdW5xdG1zemh0dXNzdHp0c3Z4dXt6dXR1eXd4e3t7fHp1fJ1YNdBnPYhZSo5dU61c
T4hkW5hnWJJqVKlpWLVpV7hzWbBoT4dpZpdsZ4h0bpd1a4J+e4V4dZZ6dotscad4aLh4Z614co1f
YchqVsh1V9FvTMZ5Zcp1a8BeQ36AfXiDd4KCfoqFe5yLfrWGebSGdMqKdOWHZ31+gXR3hGdugYh9
gn+AgoODhIuKhYWGiYeIiouLjIiHhJGOipWKhI6RjpOSjZmVjI2OkY+QkpOUlJualZWWmZeYmpuc
nJiXlJeMkLWKhbiVi6Obl7aal6ePiZ6gnqSinauhmbmhmp2eoZ+goqOjpKusq6iop7iqp7S0s7q5
uLGvrsiViNWVicicls+RjMijmdanl9KtlcSrp9m3qse7uM6zreS4ruSjmc7EuerIu8jIx9TSzdTU
09zc3NnY1tXMx+jVyPjYyufb2PDRy9zi3Prm2e/q2uTk4+zs6+vo5vjr5+vz7PXz7f707P777Pv4
5u39/O369/X19Pz29PT79fz79fX1+/32/PX8/f////nu9t/g5MC/wSH+EUNyZWF0ZWQgd2l0aCBH
SU1QACH5BAEKAP0ALAAAAAB4AGYAAAj+APsJHEiwoMGDCBMqXMiwocOHECNKnEixosWLGBOq+5ax
o8ePCsNx8oEhD8iTKC+GW5UiRQMNGvS4S0mzpkJvqVycOMGgAQSXumwKtRmvWxgVGjCAeMDgAYQH
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
}

#####################################################################################
# Tooltip
#####################################################################################

proc setTooltip {widget text} {
	if { $text != "" } {

		bind $widget <Any-Enter>    [list  .common.helptext configure -text "Tooltip: $text"]

#		bind $widget <Any-Enter>    [list after 500 [list showTooltip %W $text]]
#		bind $widget <Any-Leave>    [list after 500 [list destroy %W.tooltip]]
#		bind $widget <Any-KeyPress> [list after 500 [list destroy %W.tooltip]]
#		bind $widget <Any-Button>   [list after 500 [list destroy %W.tooltip]]
	}
}

proc showTooltip {widget text} {
	global tcl_platform
	if { [string match $widget* [winfo containing  [winfo pointerx .] [winfo pointery .]] ] == 0  } {
		return
	}
	catch { destroy $widget.tooltip }
	set scrh [winfo screenheight $widget]    ; # 1) flashing window fix
	set scrw [winfo screenwidth $widget]     ; # 1) flashing window fix
	set tooltip [toplevel $widget.tooltip -bd 1 -bg black]
	wm geometry $tooltip +$scrh+$scrw        ; # 1) flashing window fix
	wm overrideredirect $tooltip 1
	if {$tcl_platform(platform) == {windows}} { ; # 3) wm attributes...
		wm attributes $tooltip -topmost 1   ; # 3) assumes...
	}                                           ; # 3) Windows
	pack [label $tooltip.label -bg lightyellow -fg black -text $text -justify left]
	set width [winfo reqwidth $tooltip.label]
	set height [winfo reqheight $tooltip.label]
	set pointer_below_midline [expr [winfo pointery .] > [expr [winfo screenheight .] / 2.0]]                ; # b.) Is the pointer in the bottom half of the screen?
	set positionX [expr [winfo pointerx .] - round($width / 2.0)]    ; # c.) Tooltip is centred horizontally on pointer.
	set positionY [expr [winfo pointery .] + 35 * ($pointer_below_midline * -2 + 1) - round($height / 2.0)]  ; # b.) Tooltip is displayed above or below depending on pointer Y position.
	if  {[expr $positionX + $width] > [winfo screenwidth .]} {
		set positionX [expr [winfo screenwidth .] - $width]
	} elseif {$positionX < 0} {
		set positionX 0
	}
	wm geometry $tooltip [join  "$width x $height + $positionX + $positionY" {}]
	raise $tooltip
	bind $widget.tooltip <Any-Enter> {destroy %W}
	bind $widget.tooltip <Any-Leave> {destroy %W}
}

#####################################################################################
# Menu
#####################################################################################

menu .menu -tearoff 0
menu .menu.file -tearoff 0
.menu add cascade -label "File" -menu .menu.file -underline 0

	.menu.file add command -label "Connect" -command {
		connect_serial
	}
	.menu.file add command -label "Close" -command {
		close_serial
	}
	.menu.file add command -label "Load config from file..." -command {
		load_values_from_file
	}
	.menu.file add command -label "Save config to file..." -command {
		save_values2file
	}
	.menu.file add separator
	.menu.file add command -label "Exit" -command {
		exit 0
	}

menu .menu.options -tearoff 0
.menu add cascade -label "Options" -menu .menu.options -underline 0
	.menu.options add command -label "set Defaults" -command {
		set_defaults
	}
	.menu.options add command -label "Get config from EEPROM" -command {
		load_from_eeprom
	}
	.menu.options add command -label "Save config to EEPROM" -command {
		save_to_eeprom
	}

if {$HEXDATA != ""} {
	menu .menu.firmware -tearoff 0
	.menu add cascade -label "Firmware" -menu .menu.firmware -underline 0
		.menu.firmware add command -label "Upload" -command {
			close_serial
			set device [.common.device.spin get]
			set ArduinoSerial [ArduinoSerial_Init [.common.device.spin get] 57600]
			ArduinoReset
			ArduinoSerial_SendCMD $defs(STK_GET_SIGN_ON)
			if {$ArduinoTimeout != 0} {
				.bottom.status configure -text "ERROR: upload firmware"
				.bottom.status configure -background red
				update
				after 1000
			} else {
				.bottom.status configure -text "start uploading firmware"
				ArduinoSendData "0x0000" $HEXDATA
				ArduinoReset
			}
			after 500
			connect_serial
		}
}


menu .menu.help -tearoff 0
	.menu add cascade -label "Help" -menu .menu.help -underline 0
#	.menu.help add command -label "Tuning" -command {
#		show_help_tuning
#	}
	.menu.help add separator
	.menu.help add command -label "Homepage" -command {
		launchBrowser "http://code.google.com/p/brushless-gimbal/"
	}
	.menu.help add command -label "Documentation" -command {
		launchBrowser "http://code.google.com/p/brushless-gimbal/"
	}
	.menu.help add separator
	.menu.help add command -label "About..." -command {
		show_help_about
	}

. configure -menu .menu

#####################################################################################
# serialports
#####################################################################################

if {[string match "*Linux*" $tcl_platform(os)]} {
	set comports ""
	set device ""
	catch {
		set comports "[glob /dev/ttyUSB*] [glob /dev/ttyACM*]"
		set device "[lindex $comports end]"
	}
} elseif {[string match "*Windows*" $tcl_platform(os)]} {
	set comports {"com1:" "com2:" "com3:" "com4:" "com5:" "com6:" "com7:" "com8:" "com9:" "com10:" "com11:" "com12:" "com13:" "com14:" "com15:"}
	catch {
		set serial_base "HKEY_LOCAL_MACHINE\\HARDWARE\\DEVICEMAP\\SERIALCOMM"
		set values [registry values $serial_base]
		set res {}
#		foreach valueName $values {
#			set PortName "[registry get $serial_base $valueName]"
#			lappend res "$PortName"
#		}
		foreach valueName $values {
			set PortName "//./[registry get $serial_base $valueName]"
			lappend res "$PortName"
		}
		set comports $res
	}
	set device "[lindex $comports end]"
} elseif {[string match "*Darwin*" $tcl_platform(os)] || [string match "*MacOS*" $tcl_platform(os)]} {
	set comports ""
	set device ""
	catch {
		set comports [glob /dev/cu.usbserial-*]
		set device "[lindex $comports end]"
	}
}

#####################################################################################
# Global variables
#####################################################################################

set Serial 0
set LastValX 0
set LastValY 0
set chart 0
set params "gyroPitchKp gyroPitchKi gyroPitchKd gyroRollKp gyroRollKi gyroRollKd accTimeConstant angleOffsetPitch angleOffsetRoll \
            dirMotorPitch dirMotorRoll motorNumberPitch motorNumberRoll maxPWMmotorPitch maxPWMmotorRoll refVoltageBat cutoffVoltage motorPowerScale \
            rcAbsolutePitch rcAbsoluteRoll maxRCPitch maxRCRoll minRCPitch minRCRoll rcGainPitch rcGainRoll rcLPFPitch rcLPFRoll \
            rcModePPMPitch rcModePPMRoll rcModePPMAux rcModePPMFpvP rcModePPMFpvR \
            rcPinModeCH0 rcPinModeCH1 rcPinModeCH2 \
            rcChannelPitch rcChannelRoll rcChannelAux rcChannelFpvP rcChannelFpvR fpvGainPitch fpvGainRoll rcLPFPitchFpv rcLPFRollFpv \
            rcMid fTrace sTrace enableGyro enableACC axisReverseZ axisSwapXY \
            fpvFreezePitch fpvFreezeRoll maxPWMfpvPitch maxPWMfpvRoll fpvSwPitch fpvSwRoll altSwAccTime accTimeConstant2 \
            gyroCal gyrOffsetX gyrOffsetY gyrOffsetZ accOffsetX accOffsetY accOffsetZ"
 
foreach var $params {
	if {! [string match "*,*" $var]} {
		set par($var) 0
		set par($var,scale) 1
		set par($var,offset) 0
    set par($var,comboboxOptions) {}
	}
}

set enable_trace 1

set par(gyroPitchKp,scale) 1000.0
set par(gyroPitchKi,scale) 1000.0
set par(gyroPitchKd,scale) 1000.0
set par(gyroRollKp,scale) 1000.0
set par(gyroRollKi,scale) 1000.0
set par(gyroRollKd,scale) 1000.0
set par(angleOffsetPitch,scale) 100.0
set par(angleOffsetRoll,scale) 100.0
set par(rcLPFPitch,scale) 10.0
set par(rcLPFRoll,scale) 10.0
set par(rcLPFPitchFpv,scale) 10.0
set par(rcLPFRollFpv,scale) 10.0
set par(maxPWMmotorPitch,scale) 2.5
set par(maxPWMmotorRoll,scale) 2.5
set par(rcChannelPitch,offset) 1
set par(rcChannelRoll,offset) 1
set par(rcChannelAux,offset) 1
set par(rcChannelFpvP,offset) 1
set par(rcChannelFpvR,offset) 1
set par(motorNumberPitch,offset) 1
set par(motorNumberRoll,offset) 1
set par(refVoltageBat,scale) 100.0
set par(cutoffVoltage,scale) 100.0
set par(maxPWMfpvPitch,scale) 2.5
set par(maxPWMfpvRoll,scale) 2.5

#
# parameters with non-numeric values (as used by gui_combobox)
#
set par(fpvSwPitch,comboboxOptions) {"permanent On" "permanent Off" AuxSW1 AuxSW2}
set par(fpvSwPitch,offset) 1
set par(fpvSwRoll,comboboxOptions) {"permanent On" "permanent Off" AuxSW1 AuxSW2}
set par(fpvSwRoll,offset) 1
set par(altSwAccTime,comboboxOptions) {"permanent On" "permanent Off" AuxSW1 AuxSW2}
set par(altSwAccTime,offset) 1

set par(fpvFreezePitch,comboboxOptions) {Follow Freeze}
set par(fpvFreezeRoll,comboboxOptions) {Follow Freeze}


set par(rcModePPMPitch,comboboxOptions) {PWM "PPM Sum"}
set par(rcModePPMRoll,comboboxOptions) {PWM "PPM Sum"}
set par(rcModePPMFpvP,comboboxOptions) {PWM "PPM Sum"}
set par(rcModePPMFpvR,comboboxOptions) {PWM "PPM Sum"}
set par(rcModePPMAux,comboboxOptions) {PWM "PPM Sum"}

set par(rcPinModeCH0,comboboxOptions) {"disabled" "digital PWM/PPM input" "analog input"}
set par(rcPinModeCH1,comboboxOptions) {"disabled" "digital PWM input" "analog input"}
set par(rcPinModeCH2,comboboxOptions) {"disabled" "digital PWM input" "analog input"}

set CHART_SCALE 0.5
set buffer ""
set chart_count 0

set BruGiReady 0
set liveViewON 0

# trace variables for data logging
set traceVars "\
             rcPitch rcRoll rcAux rcFpvPitch rcFpvRoll \
             rcPitchValid rcRollValid rcAuxValid rcFpvPitchValid rcFpvRollValid \
             rcAuxSW1 rcAuxSW2 \
             accX accY accZ accMag \
             fpvModePitch fpvModeRoll altModeAccTime \
             estX estY estZ angleRoll anglePitch \
             gyroX gyroY gyroZ \
             accX accY accZ accMag \
             pitchAngleSet anglePitch pitchMotorDrive pitchPidError pitchErrorSum \
             rollAngleSet angleRoll rollMotorDrive rollPidError rollErrorSum \
             mpuTemp mpuI2cErrors \
             "

# initialize trace vars             
proc init_traceVar {} {
  global traceVars
  global traceVar
  
  foreach var $traceVars {
    set traceVar($var) 0
    set traceVar($var,scale) 1
    set traceVar($var,offset) 0
  }
}

init_traceVar

#####################################################################################
# Serial-Functions
#####################################################################################

proc Serial_Init {ComPort ComRate} {
	global Serial
	global chart
	catch {close $Serial}
	catch {fileevent $Serial readable ""}
	set iChannel 0
	if {[catch {
		set iChannel [open $ComPort w+]
		fconfigure $iChannel -mode $ComRate,n,8,1 -ttycontrol {RTS 1 DTR 0} -blocking FALSE
		fileevent $iChannel readable [list rd_chid $iChannel]
		.common.bottom1.message configure -text "Serial-Ok: $ComPort @ $ComRate" -background lightgrey
		.common.bottom1.linkStatus configure -background lightgrey
		.common.device.connect configure -text "Reconnect"
  }]} {
		.common.bottom1.message configure -text "Serial-Error: $ComPort @ $ComRate" -background lightgrey
		.common.bottom1.linkStatus configure -background red
		.common.device.connect configure -text "Connect"
    set chart 0
    .common.chartview.chart.fr1.button configure -text "Start Waveform View" -background lightgrey
		return 0
	}

  .common.monitor.liveViewButton configure -background lightgrey
  # init the trace display fields in any case
  init_traceDisplay
  update
  
	return $iChannel
}

proc close_serial {} {
	global Serial

#  set liveViewON 0
#  init_traceVar
#  update
#  link_send_cmd "par sTrace 0" "live monitor OFF" 300

	catch {close $Serial}
  set Serial 0
	.common.device.connect configure -text "Connect"
  .common.bottom1.linkStatus configure -background red
  .common.bottom1.message configure -text "not connected"  -background lightgrey
  set chart 0
  .common.chartview.chart.fr1.button configure -text "Start Waveform View" -background lightgrey
  .common.bottom1.bruGiStatus configure -background lightgrey
  .common.bottom.bruGiVersion configure -text "Firmware: -----"
  
  .common.monitor.liveViewButton configure -background lightgrey
  # init the trace display fields
  init_traceDisplay
  update
}

#####################################################################################
# CLI-Functions
#####################################################################################

proc connect_serial {} {
	global Serial
	global device
	global buffer

#	.common.bottom1.linkStatus configure -background yellow

  if {$Serial != 0} {
    close_serial
  }

	set device [.common.device.spin get]
	set Serial [Serial_Init $device 115200]
	if {$Serial == 0} {
		.common.bottom1.linkStatus configure -background red
		.common.bottom1.message configure -text "not connected" -background lightgrey
		return
	} else {
		.common.bottom1.linkStatus configure -background green
		.common.bottom1.message configure -text "connected" -background lightgrey
	}
	#after 9000 send_par
}

proc draw_chart {} {
	global Serial
	global chart
	
	if {$Serial == 0} {
		.common.bottom1.linkStatus configure -background red
		.common.bottom1.message configure -text "not connected" -background lightgrey
		return
	}
	if {$chart == 1} {
		set chart 0
		.common.bottom2.txlog configure -text "TX: par fTrace 0"
        	puts -nonewline $Serial "par fTrace 0\n"
		flush $Serial
		.common.chartview.chart.fr1.button configure -text "Start Waveform View" -background lightgrey
	} else {
		set chart 1
		.common.bottom2.txlog configure -text "TX: par fTrace 254"
        	puts -nonewline $Serial "par fTrace 254\n"
		flush $Serial
		.common.chartview.chart.fr1.button configure -text "Stop Waveform View" -background green
	}
}

proc send_parvar {n1 n2 op} {
	global Serial
	global par
	global enable_trace

	if {$enable_trace == 1} {
		if {$Serial == 0} {
			.common.bottom1.linkStatus configure -background red
			.common.bottom1.message configure -text "not connected" -background lightgrey
			return
		}
		if {$n2 == "dirMotorPitch" || $n2 == "dirMotorRoll"} {
			if {$par($n2) == 1} {
				.common.bottom2.txlog configure -text "TX: par $n2 -1"
				puts -nonewline $Serial "par $n2 -1\n"
			} else {
				.common.bottom2.txlog configure -text "TX: par $n2 1"
				puts -nonewline $Serial "par $n2 1\n"
			}
		} else {
			if {[llength $par($n2,comboboxOptions)] > 0} {
        set idx [lsearch $par($n2,comboboxOptions) $par($n2)]
        if {$idx >= 0} {
          .common.bottom2.txlog configure -text "TX: par $n2 [expr $idx * $par($n2,scale) - $par($n2,offset)]"
          puts -nonewline $Serial "par $n2 [expr $idx * $par($n2,scale) - $par($n2,offset)]\n"
        } else {
          .common.bottom2.txlog configure -text "TX: illegal comboboxOptions $n2 $par($n2)"
        }
      } else {
        .common.bottom2.txlog configure -text "TX: par $n2 [expr $par($n2) * $par($n2,scale) - $par($n2,offset)]"
        puts -nonewline $Serial "par $n2 [expr $par($n2) * $par($n2,scale) - $par($n2,offset)]\n"
      }
		}
		flush $Serial
		after 20
	}
}

proc send_par {} {
	global Serial
	global device
	global buffer
	global enable_trace

	set buffer ""
	if {$Serial == 0} {
		.common.bottom1.linkStatus configure -background red
		.common.bottom1.message configure -text "not connected" -background lightgrey
		return
	}
	set enable_trace 0 
    puts -nonewline $Serial "par\n"
	flush $Serial
}

proc load_values_from_file {} {
	global Serial

	set types {
		{"Text files"		{.txt}	}
		{"Text files"		{}		TEXT}
		{"All files"		*}
	}
	global selected_type
	if {![info exists selected_type]} {
		set selected_type "Tcl Scripts"
	}
	set file [tk_getOpenFile -filetypes $types -parent . -typevariable selected_type]
	if {$file != ""} {
		set fp [open $file r]
		if {$fp > 0} {
			set file_data [read $fp]
			close $fp
			foreach line "[split $file_data "\n"]" {
				if {$line != ""} {
				        puts $Serial "$line"
					flush $Serial
					after 20
				}
			}
			.common.bottom1.message configure -text "load: done" -background lightgrey
			update

		} else {
			.common.bottom1.message configure -text "load: error, filename = $file" -background red
			update
		}
	}
	after 200 send_par
}

proc save_values2file {} {
	global Serial
	global device
	global par

	set types {
		{"Text files"		{.txt}	}
		{"Text files"		{}		TEXT}
		{"All files"		*}
	}
  # reset trace parameters
  set par(sTrace) 0
  set par(fTrace) 0
  
	set file [tk_getSaveFile -filetypes $types -parent . -initialfile blg-gimbal -defaultextension .txt]
	if {$file != ""} {
		set fp [open $file w]

    set names [array names par]
    set names [lsort $names]
		foreach var $names {
      if {! [string match "*,*" $var]} {
        if {[llength $par($var,comboboxOptions)] > 0} {
          set idx [lsearch $par($var,comboboxOptions) $par($var)]
          if {$idx >= 0} {
            puts "$var $par($var) $par($var,scale) $par($var,offset)\n"
            puts -nonewline $fp "par $var [expr $idx * $par($var,scale) - $par($var,offset)]\n"
          } else {
            .common.bottom1.message configure -text "error: illegal comboboxOption $var $par($var)" -background red
          }
        } else {
          if {$var == "dirMotorPitch" || $var == "dirMotorRoll"} {
            if {$par($var) == 1} {
              puts -nonewline $fp "par $var -1\n"
            } else {
              puts -nonewline $fp "par $var 1\n"
            }
          } else {
            puts -nonewline $fp "par $var [expr $par($var) * $par($var,scale) - $par($var,offset)]\n"
          }
        }
			}
		}
		close $fp
	}
}

proc acc_cal {} {
	global Serial
	global device

  link_send_cmd "ac" "acc calibration command" 2000
}

proc acc_cal_reset {} {
	global device
  global par

	set par(accOffsetX) 0
	set par(accOffsetY) 0
	set par(accOffsetZ) 0
}

proc gyro_cal {} {
	global Serial
	global device

  link_send_cmd "gc" "gyro calibration command" 5500
}

proc gyro_cal_reset {} {
	global device
  global par

	set par(gyrOffsetX) 0
	set par(gyrOffsetY) 0
	set par(gyrOffsetZ) 0
}


proc save_to_eeprom {} {
	global Serial
	global device

  link_send_cmd "we" "saved config to EEPROM" 200
}

proc load_from_eeprom {} {
	global Serial
	global device

	update
  link_send_cmd "re" "loaded config from EEPROM" 200
}

proc set_defaults {} {
	global Serial
	global device

	update
  link_send_cmd "sd" "set config to default values" 200
}

proc set_batteryVoltage {} {
	global Serial
	global device

	update
  link_send_cmd "sbv" "read battery voltage" 200
}

# set update log variable
proc set_traceVar {name data } {
  global traceVar

  set traceVar($name) [expr ($data + $traceVar($name,offset)) / $traceVar($name,scale)]
}

# Live View On button
proc live_view {} {
	global Serial
	global device
  global liveViewON

  if {$liveViewON == 0} {
    .common.monitor.liveViewButton configure -background green
    set liveViewON 1
    update
    link_send_cmd "par sTrace 255" "live monitor ON" 200
  } else {
    .common.monitor.liveViewButton configure -background lightgrey
    set liveViewON 0
    init_traceDisplay
    update
    link_send_cmd "par sTrace 0" "live monitor OFF" 200
  }  
}

# send a command with link check
proc link_send_cmd {cmd message delay} {
	global Serial
    if {$Serial == 0} {
		.common.bottom1.linkStatus configure -background red
		.common.bottom1.message configure -text "not connected" -background lightgrey
		return 1
	}
  puts -nonewline $Serial "$cmd\n"
	flush $Serial
	.common.bottom1.message configure -text "$message" -background lightgrey
	after $delay send_par
  return 0
}

#####################################################################################
# Serial-Callback
#####################################################################################

proc rd_chid {chid} {
	global buffer
	global chart_count
	global VERSION
	global par
	global enable_trace
  global BruGiReady
 	global traceVar

  if { [eof $chid] } {
    return
  }  

	if {$chid == 0} {
		return
	}
  
#	catch {
		set ch [read $chid 1]
		if {$ch == "\r"} {
		} elseif {$ch == "\n"} {
			if {1 == 1} {
			  .common.bottom2.rxlog configure -text "$buffer"
#				update
				set var [lindex $buffer 0]
				set val [lindex $buffer 1]
        
				if {$var == "message"} {
          set message [lrange $buffer 2 end]
          if {$val == "VERSION:"} {
            .common.bottom.bruGiVersion configure -text "Firmware: ${message}"
            #.bottom.bruGiVersion configure -background lightgray
          }
          if {$val == "INFO:"} {
            .common.bottom1.message configure -text "INFO: ${message}"  -background green
            .common.bottom1.bruGiStatus configure -background green
            if {${message} == "BruGi ready"} {
              set BruGiReady 1
              after 200 send_par
            }
          }
          if {$val == "WARNING:"} {
            .common.bottom1.message configure -text "WARNING: ${message}" -background yellow
            .common.bottom1.bruGiStatus configure -background yellow
          }
          if {$val == "ERROR:"} {
            .common.bottom1.message configure -text "ERROR: ${message}"  -background red
          } 
        } elseif {$var == "traceData"} {
          if {$val == "RC"} {
            set_traceVar "rcPitch" [lindex $buffer 2]
            set_traceVar "rcRoll" [lindex $buffer 3]
            set_traceVar "rcAux" [lindex $buffer 4]
            set_traceVar "rcFpvPitch" [lindex $buffer 5]
            set_traceVar "rcFpvRoll" [lindex $buffer 6]
            set_traceVar "rcPitchValid" [lindex $buffer 7]
            set_traceVar "rcRollValid" [lindex $buffer 8]
            set_traceVar "rcAuxValid" [lindex $buffer 9]
            set_traceVar "rcFpvPitchValid" [lindex $buffer 10]
            set_traceVar "rcFpvRollValid" [lindex $buffer 11]
            set_traceVar "rcAuxSW1" [lindex $buffer 16]
            set_traceVar "rcAuxSW2" [lindex $buffer 17]
          }
          if {$val == "AUX"} {
            set_traceVar "fpvModePitch" [lindex $buffer 2]
            set_traceVar "fpvModeRoll" [lindex $buffer 3]
            set_traceVar "altModeAccTime" [lindex $buffer 4]
          }
          if {$val == "IMU"} {
            set_traceVar "estX" [lindex $buffer 2]
            set_traceVar "estY" [lindex $buffer 3]
            set_traceVar "estZ" [lindex $buffer 4]
            set_traceVar "angleRoll" [lindex $buffer 5]
            set_traceVar "anglePitch" [lindex $buffer 6]
          }
          if {$val == "GYRO"} {
            set_traceVar "gyroX" [lindex $buffer 2]
            set_traceVar "gyroY" [lindex $buffer 3]
            set_traceVar "gyroZ" [lindex $buffer 4]
          }
          if {$val == "SENSOR_ACC"} {
            set_traceVar "accX" [lindex $buffer 2]
            set_traceVar "accY" [lindex $buffer 3]
            set_traceVar "accZ" [lindex $buffer 4]
            set_traceVar "accMag" [lindex $buffer 5]
          }
          if {$val == "PID_PITCH"} {
            set_traceVar "pitchAngleSet" [lindex $buffer 2]
            set_traceVar "anglePitch" [lindex $buffer 3]
            set_traceVar "pitchMotorDrive" [lindex $buffer 4]
            set_traceVar "pitchPidError" [lindex $buffer 5]
            set_traceVar "pitchErrorSum" [lindex $buffer 6]
          }
          if {$val == "PID_ROll"} {
            set_traceVar "rollAngleSet" [lindex $buffer 2]
            set_traceVar "angleRoll" [lindex $buffer 3]
            set_traceVar "rollMotorDrive" [lindex $buffer 4]
            set_traceVar "rollPidError" [lindex $buffer 5]
            set_traceVar "rollErrorSum" [lindex $buffer 6]
          }
          if {$val == "MPU"} {
            set_traceVar "mpuTemp" [lindex $buffer 2]
            set_traceVar "mpuI2cErrors" [lindex $buffer 3]
          }
          if {$val == "ACC2"} {
            set chart 1
            global LastValX
            global LastValY
            global CHART_SCALE
            set TEST [lindex $buffer 1]
            set ValX [expr [lindex $buffer 2] / 1000.0]
            set ValY [expr [lindex $buffer 3] / 1000.0]
            if {($TEST == "ACC2") && [string is double -strict $ValX] && [string is double -strict $ValY]} {
              incr chart_count 1
              if {$chart_count >= 450} {
                set chart_count 0
              }
              .common.chartview.chart.chart1 delete "line_$chart_count"
              .common.chartview.chart.chart1 create line $chart_count [expr 100 - ($LastValX / 2 * $CHART_SCALE + 50)] [expr $chart_count + 1] [expr 100 - ($ValX * $CHART_SCALE / 2 + 50)] -fill orange -tags "line_$chart_count"
              .common.chartview.chart.chart1 create line $chart_count [expr 100 - ($LastValY / 2 * $CHART_SCALE + 50)] [expr $chart_count + 1] [expr 100 - ($ValY * $CHART_SCALE / 2 + 50)] -fill green -tags "line_$chart_count"
              .common.chartview.chart.chart1 delete "pos"
              .common.chartview.chart.chart1 create line [expr $chart_count + 1] 0 [expr $chart_count + 1] 100 -fill yellow -tags "pos"
              .common.chartview.chart.chart1 create text 5 10 -text "Pitch: $ValX" -anchor w -fill orange -tags "pos"
              .common.chartview.chart.chart1 create text 5 25 -text "Roll:  $ValY" -anchor w -fill green -tags "pos"
              .common.chartview.chart.chart1 create text 5 90 -text "Scale:  $CHART_SCALE" -anchor w -fill green -tags "pos"
              set LastValX $ValX
              set LastValY $ValY
            }
          }
        } elseif {$var == "dirMotorPitch" || $var == "dirMotorRoll"} {
					if {$val == -1} {
						set par($var) 1
					} else {
						set par($var) 0
					}
				} elseif {[info exists par($var,scale)]} {
          # par paramters
          if {[llength $par($var,comboboxOptions)] > 0} {
            # text
            set idx [expr ($val + $par($var,offset)) / $par($var,scale)]
            if {$idx >= 0} {
              set parValue [lindex $par($var,comboboxOptions) $idx]
              set par($var) $parValue
            } else {
              set par($var) "invalid combobox text"
            }
          } else {
            # numeric
            set par($var) [expr ($val + $par($var,offset)) / $par($var,scale)]
          }
				} else {
					set enable_trace 1
				}
			}
			set buffer ""
		} else {
			append buffer $ch
		}
#	}
  # get parameters when "INFO: Brugi ready" has been received
}

#####################################################################################
# Arduino-Flashloader
#####################################################################################

set defs(STK_OK)		"0x10"
set defs(STK_INSYNC)		"0x14"
set defs(STK_GET_SIGN_ON)	"0x30"
set defs(STK_LOAD_ADDRESS)	"0x55"
set defs(STK_READ_PAGE)		"0x74"
set defs(STK_WRITE_PAGE)	"0x64"
set defs(CRC_EOP)		"0x20"
set ArduinoTimeout 0

proc ArduinoSerial_Init {ComPort ComRate} {
	global ArduinoSerial
	catch {close $ArduinoSerial}
	set iChannel 0
	if {[catch {
		set iChannel [open $ComPort w+]
		fconfigure $iChannel -mode $ComRate,n,8,2 -translation binary -ttycontrol {RTS 1 DTR 0} -blocking FALSE
		.bottom.status configure -text "ArduinoSerial-OK: $ComPort @ $ComRate"
		update
	}]} {
		.bottom.status configure -text "ArduinoSerial-Error: $ComPort @ $ComRate"
		update
	}
	return $iChannel
}

proc ArduinoWait_reply {} {
	global ArduinoSerial
	global ArduinoTimeout
	global defs
	set num 0
	set num2 0
	set ch [read $ArduinoSerial 1]
	binary scan $ch c num
	set counter 0
	while {$num == 0 && $counter < 1000} {
		set ch [read $ArduinoSerial 1]
		binary scan $ch c num
		after 10
		incr counter
	}
	if {$counter >= 1000} {
		.bottom.status configure -text "#### timeout ####"
		update
		set ArduinoTimeout 1
		after 1000
		return
	}
	binary scan $ch c num
	set ch2 [read $ArduinoSerial 1]
	binary scan $ch2 c num2
	set counter 0
	while {$num2 == 0 && $counter < 1000} {
		set ch2 [read $ArduinoSerial 1]
		binary scan $ch2 c num2
		after 10
		incr counter
	}
	if {$counter >= 1000} {
		.bottom.status configure -text "#### timeout ####"
		update
		set ArduinoTimeout 1
		after 1000
		return
	}
	binary scan $ch2 c num2
	set ret "[format 0x%02x $num]"
	set ret2 "[format 0x%02x $num2]"
	if {$ret == $defs(STK_INSYNC) && $ret2 == $defs(STK_OK)} {
#		puts "	STK_INSYNC"
	} else {
		puts "Error: "
		puts "	< $ret"
		puts "	< $ret2"
		.bottom.status configure -text "Flash-Error"
		update
		after 1000
#		return
	}
}

proc ArduinoSerial_SendCMD {CMD} {
	global ArduinoSerial
	global defs
	puts -nonewline $ArduinoSerial "[binary format c $CMD]"
	flush $ArduinoSerial
	puts -nonewline $ArduinoSerial "[binary format c $defs(CRC_EOP)]"
	flush $ArduinoSerial
	ArduinoWait_reply
}

proc ArduinoReset {} {
	global ArduinoSerial
	fconfigure $ArduinoSerial -ttycontrol {RTS 0}
	flush $ArduinoSerial
	after 200
	fconfigure $ArduinoSerial -ttycontrol {RTS 1}
	flush $ArduinoSerial
	after 200
}

proc ArduinoSendWORD {WORD} {
	global ArduinoSerial
	global defs
	set WORD [expr $WORD / 2]
	set WORD_BYTE2 "0x[string range [format %04x $WORD] 2 3]"
	set WORD_BYTE1 "0x[string range [format %04x $WORD] 0 1]"
	puts -nonewline $ArduinoSerial "[binary format c $WORD_BYTE2]"
	flush $ArduinoSerial
	puts -nonewline $ArduinoSerial "[binary format c $WORD_BYTE1]"
	flush $ArduinoSerial
}

proc ArduinoSetAddr {ADDR} {
	global ArduinoSerial
	global defs
	puts -nonewline $ArduinoSerial "[binary format c $defs(STK_LOAD_ADDRESS)]"
	flush $ArduinoSerial
	ArduinoSendWORD "$ADDR"
	puts -nonewline $ArduinoSerial "[binary format c $defs(CRC_EOP)]"
	flush $ArduinoSerial
	ArduinoWait_reply
}

proc ArduinoSendData {START_ADDR file_data} {
	global ArduinoSerial
	global defs
	set BUFFER ""
	set COUNT 0
	if {$file_data != ""} {
		set MAX_ADDR [expr [llength $file_data]]
		foreach BYTE $file_data {
			lappend BUFFER "$BYTE"
			if {$COUNT == 127} {
				.bottom.status configure -text "write: [format 0x%02x $START_ADDR] ([expr [format 0x%x $START_ADDR] * 100 / [format 0x%x $MAX_ADDR]]%)"
				update
				ArduinoSetAddr "[format 0x%x $START_ADDR]"
				puts -nonewline $ArduinoSerial "[binary format c $defs(STK_WRITE_PAGE)]"
				flush $ArduinoSerial
				puts -nonewline $ArduinoSerial "[binary format c "0x0"]"
				flush $ArduinoSerial
				puts -nonewline $ArduinoSerial "[binary format c "0x80"]"
				flush $ArduinoSerial
				puts -nonewline $ArduinoSerial "F"
				flush $ArduinoSerial
				foreach SEND_BYTE $BUFFER {
					puts -nonewline $ArduinoSerial "[binary format c $SEND_BYTE]"
					flush $ArduinoSerial
				}
				puts -nonewline $ArduinoSerial "[binary format c $defs(CRC_EOP)]"
				flush $ArduinoSerial
				ArduinoWait_reply
				incr START_ADDR 128
				set BUFFER ""
				set COUNT 0
			} else {
				incr COUNT
			}
		}
		set NUM 0
		while {$NUM < [expr 128 - $COUNT]} {
			lappend BUFFER "0xFF"
			incr NUM
		}
		.bottom.status configure -text "write: [format 0x%02x $START_ADDR] ([expr [format 0x%x $START_ADDR] * 100 / [format 0x%x $MAX_ADDR]]%)"
		update
		ArduinoSetAddr "[format 0x%x $START_ADDR]"
		puts -nonewline $ArduinoSerial "[binary format c $defs(STK_WRITE_PAGE)]"
		flush $ArduinoSerial
		puts -nonewline $ArduinoSerial "[binary format c "0x0"]"
		flush $ArduinoSerial
		puts -nonewline $ArduinoSerial "[binary format c "0x80"]"
		flush $ArduinoSerial
		puts -nonewline $ArduinoSerial "F"
		flush $ArduinoSerial
		foreach SEND_BYTE $BUFFER {
			puts -nonewline $ArduinoSerial "[binary format c $SEND_BYTE]"
			flush $ArduinoSerial
		}
		puts -nonewline $ArduinoSerial "[binary format c $defs(CRC_EOP)]"
		flush $ArduinoSerial
		ArduinoWait_reply
		.bottom.status configure -text "write: done (100%)"
		update
	} else {
		.bottom.status configure -text "error no data found"
		update
		after 1000
	}
}

proc ArduinoConvertData {file_data} {
	global ArduinoSerial
	global defs
	set BUFFER ""
	if {$file_data != ""} {
		foreach BYTE [split $file_data ""] {
			binary scan $BYTE c num
			if {$num < 0} {
				set num [expr $num + 256]
			}
			lappend BUFFER "[format 0x%x $num]"
		}
		return $BUFFER
	} else {
		.bottom.status configure -text "error no data found"
		update
		after 1000
	}
}

proc ArduinoSendBIN {START_ADDR BINFILE} {
	global ArduinoSerial
	set binfile [open $BINFILE r]
	fconfigure $binfile -translation binary
	set file_data [read $binfile]
	close $binfile
	if {$file_data != ""} {
		ArduinoSendData $START_ADDR [ArduinoConvertData $file_data]
	} else {
		.bottom.status configure -text "error loading BIN-File"
		update
		after 1000
	}
}

proc ArduinoExportBIN {BINFILE} {
	set binfile [open $BINFILE r]
	fconfigure $binfile -translation binary
	set file_data [read $binfile]
	close $binfile
	if {$file_data != ""} {
		return "[ArduinoConvertData $file_data]"
	} else {
		.bottom.status configure -text "error loading BIN-File"
		update
		after 1000
	}
}

#####################################################################################
# GUI-Helper-Functions
#####################################################################################

set tline 0
proc show_textline {wid line} {
	global tline
	label $wid.tline_$tline -anchor nw -text "$line"
	pack $wid.tline_$tline -side top -expand yes -fill x
	incr tline
}

proc motorNumberPitch_check {n1 n2 op} {
	global par
	catch {
		if {$par(motorNumberPitch) == 1} {
			set par(motorNumberRoll) 2
		} else {
			set par(motorNumberRoll) 1
		}
	}
}
trace variable par(motorNumberPitch) w motorNumberPitch_check

proc motorNumberRoll_check {n1 n2 op} {
	global par
	catch {
		if {$par(motorNumberRoll) == 1} {
			set par(motorNumberPitch) 2
		} else {
			set par(motorNumberPitch) 1
		}
	}
}
trace variable par(motorNumberRoll) w motorNumberRoll_check

proc show_help {helptext} {
	tk_messageBox -icon info -message "$helptext" -type ok -parent .
}

proc gui_slider {wid variable min max step title tooltiptext helptext} {
	global par
	label $wid
	pack $wid -side top -expand yes -fill x
	setTooltip $wid "$tooltiptext"

		label $wid.label -text "$title" -width 14 -anchor w
		pack $wid.label -side left -expand no -fill x

		frame $wid.frame
		pack $wid.frame -side left -expand yes -fill x

			eval button $wid.frame.help -text \"?\" -width 1 -command \{show_help \"$helptext\"\}
			pack $wid.frame.help -side right -expand no -fill none

			spinbox $wid.frame.spin -from $min -to $max -increment $step -width 10 -textvariable par($variable) -width 4
			pack $wid.frame.spin -side right -expand yes -fill x

			scale $wid.frame.scale -orient horizontal -from $min -to $max -showvalue 0 -resolution $step -variable par($variable)
			pack $wid.frame.scale -side right -expand yes -fill x
}

proc gui_spin {wid variable min max step title tooltiptext helptext} {
	global par
	frame $wid
	pack $wid -side top -expand yes -fill x
	setTooltip $wid "$tooltiptext"

		label $wid.label -text "$title" -width 14 -anchor w
		pack $wid.label -side left -expand no -fill x

		frame $wid.frame
		pack $wid.frame -side left -expand yes -fill x

			eval button $wid.frame.help -text \"?\" -width 1 -command \{show_help \"$helptext\"\}
			pack $wid.frame.help -side right -expand no -fill none

			set diff [expr $max - $min]
			if {$diff <= 8} {
				set options ""
				set num $min
				while {$num <= $max}  {
					set options "$options $num"
					incr num
				}
				ttk::combobox $wid.frame.spin -textvariable par($variable) -state readonly -values $options
			} else {
				spinbox $wid.frame.spin -from $min -to $max -increment $step -width 10 -textvariable par($variable) -width 4
			}
			pack $wid.frame.spin -side right -expand yes -fill x
}

proc gui_combobox {wid variable title tooltiptext helptext} {
	global par
  
  set comboboxOptions $par($variable,comboboxOptions)
  
	frame $wid
	pack $wid -side top -expand yes -fill x
	setTooltip $wid "$tooltiptext"

		label $wid.label -text "$title" -width 14 -anchor w
		pack $wid.label -side left -expand no -fill x

		frame $wid.frame
		pack $wid.frame -side left -expand yes -fill x

			eval button $wid.frame.help -text \"?\" -width 1 -command \{show_help \"$helptext\"\}
			pack $wid.frame.help -side right -expand no -fill none
      
      ttk::combobox $wid.frame.spin -textvariable par($variable) -state readonly -values $comboboxOptions
			pack $wid.frame.spin -side right -expand yes -fill x
}


proc gui_check {wid variable title title2 tooltiptext helptext} {
	global par
	frame $wid
	pack $wid -side top -expand yes -fill x
	setTooltip $wid "$tooltiptext"

		label $wid.label -text "$title" -width 14 -anchor w
		pack $wid.label -side left -expand no -fill x

		frame $wid.frame
		pack $wid.frame -side left -expand yes -fill x

			checkbutton $wid.frame.check -text "$title2" -variable par($variable) -relief flat -anchor w
			pack $wid.frame.check -side left -expand yes -fill x

			eval button $wid.frame.help -text \"?\" -width 1 -command \{show_help \"$helptext\"\}
			pack $wid.frame.help -side left -expand no -fill none
}

proc gui_radio {wid variable options title tooltiptext helptext} {
	global par
	frame $wid
	pack $wid -side top -expand yes -fill x
	setTooltip $wid "$tooltiptext"

		label $wid.label -text "$title" -width 14 -anchor w
		pack $wid.label -side left -expand no -fill x

		frame $wid.frame
		pack $wid.frame -side left -expand yes -fill x

			foreach option $options  {
				set option_title [lindex $option 0]
				set option_value [lindex $option 1]
				radiobutton $wid.frame.check_$option -text "$option_title" -value $option_value -variable par($variable) -relief flat
				pack $wid.frame.check_$option -side left -expand yes -fill x
			}

			eval button $wid.frame.help -text \"?\" -width 1 -command \{show_help \"$helptext\"\}
			pack $wid.frame.help -side left -expand no -fill none
}

proc gui_button {wid title tooltiptext command} {
	eval button $wid -text \"$title\" -width 14 -command $command
	pack $wid -side left -expand yes -fill x
	setTooltip $wid "$tooltiptext"
}

proc disable_all {path} {
    catch {$path configure -state disabled}
    foreach child [winfo children $path] {
        disable_all $child
    }
}

proc enable_all {path} {
    catch {$path configure -state normal}
    foreach child [winfo children $path] {
        enable_all $child
    }
}

proc launchBrowser url {
	global tcl_platform
	switch $tcl_platform(os) {
		Darwin {
		  set command [list open $url]
		}
		HP-UX -
		Linux  -
		SunOS {
		  foreach executable {firefox mozilla netscape iexplorer opera lynx w3m links epiphany galeon konqueror mosaic amaya browsex elinks} {
		    set executable [auto_execok $executable]
		    if [string length $executable] {
		      set command [list $executable $url &]
		      set command [list $executable $url]
		      break
		    }
		  }
		}
		{Windows 95} -
		{Windows NT} {
		  set command "[auto_execok start] {} [list $url]"
		}
	}
	if [info exists command] {
		if [catch {exec {*}$command &} err] {
		  tk_messageBox -icon error -message "error '$err' with '$command'"
		}
	} else {
		tk_messageBox -icon error -message "Please tell CL that ($tcl_platform(os), $tcl_platform(platform)) is not yet ready for browsing."
	}
}

proc gui_monitor {wid variable title sw bar } {
	global traceVar
	frame $wid
	pack $wid -side top -expand yes -fill x

  label $wid.label -text "$title" -width 14 -anchor w
	pack $wid.label -side left -expand no -fill x

  if {$sw != "no"} {
    label $wid.sw  -relief sunken -text "$sw" -anchor w
    pack $wid.sw -side left -expand no -fill x
  }
  label $wid.value -relief sunken -text "$traceVar($variable)" -width 5
  pack $wid.value -side left -expand no -fill x

  if {$bar != "no"} {
    canvas $wid.bar -width 200 -height 17 -bd 1 -relief groove -highlightt 0
    $wid.bar create rectangle 0 0 0 17 -tags bar -fill navy
    pack $wid.bar -padx 5 -pady 2 -side left -expand no -fill x
  }
  
}

proc gui_monitor_update_color {wid color} {
  $wid configure -background $color
}

proc gui_monitor_update {wid val} {
  $wid.value configure -text "$val"
}

proc gui_monitor_update_bar {wid val min max} {
  $wid.value configure -text "$val"
	$wid.bar coords bar 0 0 [expr {int(($val-$min)/($max-$min) * 200.0 )} ] 17
}

proc gui_monitor_update_bar_color {wid color} {
  $wid.value configure -background $color
  $wid.bar itemconfigure bar -fill $color
}
#####################################################################################
# the GUI
#####################################################################################
wm title . "Brushless-Gimbal-Tool ($VERSION)"
  
proc update_mpu {n1 n2 op} {
        global par
        .note.general.settings.sensor.img.canv delete "arrows"
        if {$par(axisSwapXY) == 1} {
                .note.general.settings.sensor.img.canv create line 18 38 60 15 -fill green -arrow last -tag arrows
                .note.general.settings.sensor.img.canv create line 18 38 50 75 -fill red -arrow last -tag arrows
                .note.general.settings.sensor.img.canv create text 55 75 -text "X" -fill red -tag arrows
                .note.general.settings.sensor.img.canv create text 65 15 -text "Y" -fill green -tag arrows
        } else {
                .note.general.settings.sensor.img.canv create line 18 38 60 15 -fill red -arrow last -tag arrows
                .note.general.settings.sensor.img.canv create line 18 38 50 75 -fill green -arrow last -tag arrows
                .note.general.settings.sensor.img.canv create text 65 15 -text "X" -fill red -tag arrows
                .note.general.settings.sensor.img.canv create text 55 75 -text "Y" -fill green -tag arrows
        }
        if {$par(axisReverseZ) == 1} {
                .note.general.settings.sensor.img.canv create line 18 38 18 100 -fill blue -arrow last -tag arrows
                .note.general.settings.sensor.img.canv create text 23 100 -text "Z" -fill blue -tag arrows
        } else {
                .note.general.settings.sensor.img.canv create line 18 38 18 5 -fill blue -arrow last -tag arrows
                .note.general.settings.sensor.img.canv create text 23 7 -text "Z" -fill blue -tag arrows
        }
}

trace variable par(axisReverseZ) w update_mpu
trace variable par(axisSwapXY) w update_mpu


ttk::notebook .note
pack .note -fill both -side left -expand no -fill both -padx 2 -pady 3

  ttk::frame .note.general
  .note add .note.general -text "Settings"

    label .note.general.image -relief flat -anchor center -image "logo"
    pack .note.general.image -side top -fill none -expand no

    frame .note.general.settings 
    pack .note.general.settings -side top -expand yes -fill both

      labelframe .note.general.settings.power -text "Motor Power"
      pack .note.general.settings.power -side left -expand yes -fill both
       
        gui_check .note.general.settings.power.motorPowerScale  motorPowerScale   "Power Scale" "On" "compensate for battery voltage changes" "config.motorPowerScale: motor power is compensated for battery voltage changes, e.g. when battery voltage drops during operation, needs a 1k to 2k2 voltage divider from Ubat to input A3 (Multi)"
        gui_spin .note.general.settings.power.refVoltageBat   refVoltageBat   6 20 0.1 "Battery Voltage"  "refVoltageBat" "config.refVoltageBat: this is the reference battery voltage, at which control loop parameters (P,I,D, PWM) have been set"
        gui_spin .note.general.settings.power.cutoffVoltage   cutoffVoltage   6 20 0.1 "Cutoff Voltage"  "cutoffVoltage" "config.cutoffVoltage: this the minium battery voltage, motors are disabled when battery voltage drops below this voltage"
        gui_button .note.general.settings.power.setrefVoltageBat "Get Battery Voltage" "get actual battery voltage and use it as reference in variable config.refVoltageBat" set_batteryVoltage
    
      labelframe .note.general.settings.sensor -text "Sensor"
      pack .note.general.settings.sensor -side left -expand no -fill both

        frame .note.general.settings.sensor.set
        pack .note.general.settings.sensor.set -side left -expand yes -fill both

          gui_check .note.general.settings.sensor.set.axisReverseZ axisReverseZ "Reverse Z-axis" "reversed" "Set Sensor Orientation: 0=sensor mounted with component side up, 1=upside down" "Set Sensor Orientation Z-Axis: 0=sensor mounted with component side up, 1=sensor mounted upside down"
          gui_check .note.general.settings.sensor.set.axisSwapXY axisSwapXY "Swap XY-axis" "swapped" "Set Sensor Orientation XY-Axis: 0=normal, 1=swap X/Y" "Set Sensor Orientation XY-Axis: 0=normal, 1=functions of X/Y axis are exchanged"
          gui_slider .note.general.settings.sensor.set.accTimeConstant accTimeConstant 1 20 1 "ACC Time Const"  "ACC Time Constant(sec)" "tconfig.accTimeConstant: time constant of ACC complementary filter.  Controls how fast the gimbal follows ACC (sec)"
          gui_check .note.general.settings.sensor.set.enableGyro enableGyro "Gyro Update" "enabled" "Gyro update" "config.enableGyro: enable gyro update: 0=do not use gyro for attitude calcualtion, just for test and adjustment purposes"
          gui_check .note.general.settings.sensor.set.enableACC enableACC "ACC Update" "enabled" "ACC update" "config.enableACC: enable ACC update: 0=do not use ACC for attitude calculation, just for test and adjustment purposes"
 
          labelframe .note.general.settings.sensor.set.mpuMonitor
          pack .note.general.settings.sensor.set.mpuMonitor -side bottom -expand no -fill x
            gui_monitor .note.general.settings.sensor.set.mpuMonitor.mpuI2cErrors mpuI2cErrors "I2C Errors" no no
            gui_monitor .note.general.settings.sensor.set.mpuMonitor.mpuTemp mpuTemp "Temperature" no no
      
        frame .note.general.settings.sensor.img
          pack .note.general.settings.sensor.img -side left -expand no -fill none

          canvas .note.general.settings.sensor.img.canv -relief raised -width 120 -height 120
          pack .note.general.settings.sensor.img.canv -side left
          .note.general.settings.sensor.img.canv create image 0 0 -anchor nw -image sensor
          update_mpu 0 0 0
                                
    labelframe .note.general.buttons -text "Config Parameters"
    pack .note.general.buttons -side top -expand no -fill both

      frame .note.general.buttons.line1
      pack .note.general.buttons.line1 -side top -expand no -fill x

        gui_button .note.general.buttons.line1.defaults "Set Defaults" "set defaults values" set_defaults
        gui_button .note.general.buttons.line1.load_from_eeprom "Get from EEPROM" "get values from EEPROM into board and gui" load_from_eeprom
        gui_button .note.general.buttons.line1.save_to_eeprom "Save to EEPROM" "save values from board into EEPROM" save_to_eeprom

      frame .note.general.buttons.line2
      pack .note.general.buttons.line2 -side top -expand no -fill x

        gui_button .note.general.buttons.line2.load_from_file "Load from File" "load values from file into board and gui" load_values_from_file
        gui_button .note.general.buttons.line2.save2file "Save to File" "save values from gui into file" save_values2file

  ttk::frame .note.pitch
  .note add .note.pitch -text "PID Pitch"

    labelframe .note.pitch.pid -text "PID" -padx 10 -pady 10
    pack .note.pitch.pid -side top -expand no -fill x

      gui_slider .note.pitch.pid.p gyroPitchKp 0 150 0.1 "P" "P-Value" "config.gyroPitchKp: P-Value"
      gui_slider .note.pitch.pid.i gyroPitchKi 0 500 0.1 "I" "I-Value" "config.gyroPitchKi: I-Value"
      gui_slider .note.pitch.pid.d gyroPitchKd 0 150 0.1 "D" "D-Value" "config.gyroPitchKd: D-Value"

    labelframe .note.pitch.hw -text "Motor" -padx 10 -pady 10
    pack .note.pitch.hw -side top -expand no -fill x

      gui_radio .note.pitch.hw.number motorNumberPitch "{Motor-1 1} {Motor-2 2}" "Port-Number"  "Output-Port-Number" "config.motorNumberPitch: this is the motor output port for pitch"
      gui_check .note.pitch.hw.dir   dirMotorPitch            "Direction"     "reverse" "Motor-Direction" "config.dirMotorPitch: reverse motor direction, care should be taken to set the proper motor direction, otherwise the PID controller may show unexpected behaviour"
      gui_slider .note.pitch.hw.maxpwm maxPWMmotorPitch 0 100 0.1 "max PWM (%)" "maximum Motor-PWM" "config.maxPWMmotorPitch: motor power setting"

  ttk::frame .note.roll
  .note add .note.roll -text "PID Roll"

    labelframe .note.roll.pid -text "PID" -padx 10 -pady 10
    pack .note.roll.pid -side top -expand no -fill x

      gui_slider .note.roll.pid.p gyroRollKp 0 150 0.1 "P" "P-Value" "config.gyroRollKp: P-Value"
      gui_slider .note.roll.pid.i gyroRollKi 0 500 0.1 "I" "I-Value" "config.gyroRollKi: I-Value"
      gui_slider .note.roll.pid.d gyroRollKd 0 150 0.1 "D" "D-Value" "config.gyroRollKd: D-Value"

    labelframe .note.roll.hw -text "Motor" -padx 10 -pady 10
    pack .note.roll.hw -side top -expand no -fill x

      gui_radio .note.roll.hw.number motorNumberRoll "{Motor-1 1} {Motor-2 2}" "Port-Number"  "Output-Port-Number" "config.motorNumberRoll: this is the motor output port for roll"
      gui_check .note.roll.hw.dir   dirMotorRoll            "Direction"     "reverse" "Motor-Direction" "config.dirMotorRoll: reverse motor direction, care should be taken to set the proper motor direction, otherwise the PID controller may show unexpected behaviour"
      gui_slider .note.roll.hw.maxpwm maxPWMmotorRoll 0 100 0.1 "max PWM (%)" "maximum Motor-PWM" "config.maxPWMmotorRoll: motor power setting"

  ttk::frame .note.pitchRC
  .note add .note.pitchRC -text "RC Pitch"

    labelframe .note.pitchRC.set
    pack .note.pitchRC.set -side top -expand yes -fill both

      labelframe .note.pitchRC.set.rc -text "RC" -padx 10 -pady 10
      pack .note.pitchRC.set.rc -side left -expand yes -fill both
        gui_combobox .note.pitchRC.set.rc.rcModePPMPitch rcModePPMPitch      "RC PPM/PWM" "Mode of RC input, PPM sum oder single PWM RC inputs on A1/A2" "config.rcModePPM: PPM sum oder single PWM RC inputs on A0/A1/A2: PPM sum input on A2 or single RC PWM inputs on A2=Ch1, A1=Ch2, A0=Ch3"
        gui_spin   .note.pitchRC.set.rc.rcChannelPitch rcChannelPitch 0 16 1 "RC Channel #"  "rcChannelPitch" "config.rcChannelPitch: RC channel number for RC pitch, legal values 1..16 in PPM mode, 1..3 in PWM mode, 0=OFF (disabled)"
        gui_check  .note.pitchRC.set.rc.rcAbsolute rcAbsolutePitch           "RC Abs/Prop" "Absolute" "Absolute or Incremental RC control" "config.rcAbsolute: Absolute or Incremental RC control, Absolute: gimbal position follows RC transmitters directly, Proportional: RC controls the gimbal speed, thus in RC stick in center position (1500us) gimbal stops moving, where as the gimbal starts moving if stick is moved"
        gui_slider .note.pitchRC.set.rc.rcGain rcGainPitch -200 200.0 0.1    "RC Gain" "RC gain" "config.rcGain: RC Gain in Proportional mode: specifies the gain of the RC channel, larger values increas the speed of the gimbal movement"
        gui_slider .note.pitchRC.set.rc.rcLPF  rcLPFPitch 0.1 20 0.1         "RC Low Pass" "RC low pass filter" "config.rcLPF: RC low pass filter in Absolute mode: specifies speed of gimbal movement (sec)"
        gui_slider .note.pitchRC.set.rc.rcmin  minRCPitch -140 140 1         "RC min"  "minimum RC Angle" "config.minRCPitch: the amount or rotation your motor will make on that axis"
        gui_slider .note.pitchRC.set.rc.rcmax  maxRCPitch -140 140 1         "RC max"  "maximum RC Angle" "config.maxRCPitch: the amount or rotation your motor will make on that axis"
        gui_slider .note.pitchRC.set.rc.aop angleOffsetPitch -120 120 0.1    "Zero Offset" "Zero Offset" "config.angleOffsetPitch: offset adjust for pitch zero position (deg)"

      labelframe .note.pitchRC.set.fpv -text "FPV Activation" -padx 10 -pady 10
      pack .note.pitchRC.set.fpv -side top -expand no -fill x
        gui_combobox .note.pitchRC.set.fpv.fpvSw fpvSwPitch                  "FPV Switch" "select FPV control switch" "config.fpvSwPitch: RC Switch for FPV mode, legal values -1=always on, 0=off, 1=auxSW1, 2=auxSW2"
        gui_combobox .note.pitchRC.set.fpv.fpvFreezePitch fpvFreezePitch     "FPV Mode"   "select FPV mode" "config.fpvFreezePitch: select between theses modes, Follow and Freeze. In Follow mode the camera position follows the FPV RC channel coming from the flight control. The Freeze mode can be used for light weigth gimbals. During thos mode the control loop is stopped and the motor drive is frozen at the current position"

      labelframe .note.pitchRC.set.fpvMx -text "FPV Follow Mode Parameters" -padx 10 -pady 10
      pack .note.pitchRC.set.fpvMx -side top -expand no -fill x
        gui_combobox .note.pitchRC.set.fpvMx.rcModePPMFpv rcModePPMFpvP            "PPM/PWM" "Mode of RC input, PPM sum oder single PWM RC inputs on A1/A2" "config.rcModePPM: PPM sum oder single PWM RC inputs on A0/A1/A2: PPM sum input on A2 or single RC PWM inputs on A2=Ch1, A1=Ch2, A0=Ch3"
        gui_spin   .note.pitchRC.set.fpvMx.rcChannelFpv rcChannelFpvP 0 16 1       "RC Channel #"  "rcChannelFPV" "config.rcChannelFpvPitch: RC channel number for RC Aux Switch auxSW1/auxSW2, legal values 1..16 in PPM mode, 1..3 in PWM mode, 0=OFF (disabled)"
        gui_slider .note.pitchRC.set.fpvMx.fpvGain fpvGainPitch -100 100.0 0.1     "FPV gain" "FPV gain" "config.fpvGainPitch: Gain of FPV channel: specifies the gain of the FPV channel, change sign to reverse direction"
        gui_slider .note.pitchRC.set.fpvMx.rcLPFPitchFpv rcLPFPitchFpv 0.1 20 0.1  "FPV Low Pass" "FPV low pass filter" "config.rcLPFPitchFpv: RC low pass filter constant(sec)"

      labelframe .note.pitchRC.set.fpvFreeze -text "FPV Freeze Mode Parameters" -padx 10 -pady 10
      pack .note.pitchRC.set.fpvFreeze -side top -expand no -fill x
        gui_slider .note.pitchRC.set.fpvFreeze.maxPWMfpvPitch maxPWMfpvPitch 0 100 0.1 "motor PWM (%)" "alternate motor power in fpv freeze mode" "config.maxPWMfpvPitch: during fpv freeze mode, this power setting is used to increase torque"

    labelframe .note.pitchRC.monitor -text "RC Monitor"
    pack .note.pitchRC.monitor -side top -expand yes -fill both
      gui_monitor .note.pitchRC.monitor.rcPitch rcPitch "RC Pitch" "SW FPV" bar
      gui_monitor .note.pitchRC.monitor.rcFpvPitch rcFpvPitch "FPV Pitch" "SW FPV" bar
    

  ttk::frame .note.rollRC
  .note add .note.rollRC -text "RC Roll"

    labelframe .note.rollRC.set
    pack .note.rollRC.set -side top -expand yes -fill both

      labelframe .note.rollRC.set.rc -text "RC" -padx 10 -pady 10
      pack .note.rollRC.set.rc -side left -expand yes -fill both
        gui_combobox .note.rollRC.set.rc.rcModePPMRoll rcModePPMRoll       "RC PPM/PWM" "Mode of RC input, PPM sum oder single PWM RC inputs on A1/A2" "config.rcModePPM: PPM sum oder single PWM RC inputs on A0/A1/A2: PPM sum input on A2 or single RC PWM inputs on A2=Ch0, A1=Ch1, A0=Ch3"
        gui_spin   .note.rollRC.set.rc.rcChannelRoll rcChannelRoll 0 16 1  "RC Channel #"  "rcChannelRoll" "config.rcChannelRoll: RC channel number for RC roll, legal values 1..16 in PPM mode, 1..3 in PWM mode, 0=OFF (disabled)"
        gui_check  .note.rollRC.set.rc.rcAbsolute rcAbsoluteRoll           "RC Abs/Prop" "Absolute" "Absolute or Incremental RC control" "config.rcAbsolute: Absolute or Incremental RC control, Absolute: gimbal position follows RC transmitters directly, Proportional: RC controls the gimbal speed, thus in RC stick in center position (1500us) gimbal stops moving, where as the gimbal starts moving if stick is moved"
        gui_slider .note.rollRC.set.rc.rcGain rcGainRoll -200 200.0 0.1    "RC Gain" "RC gain" "config.rcGain: RC Gain in Proportional mode: specifies the gain of the RC channel, larger values increas the speed of the gimbal movement"
        gui_slider .note.rollRC.set.rc.rcLPF  rcLPFRoll 0.1 20 0.1         "RC Low Pass" "RC low pass filter" "config.rcLPF: RC low pass filter in Absolute mode: specifies speed of gimbal movement (sec)"
        gui_slider .note.rollRC.set.rc.rcmin  minRCRoll -50 50 1           "RC min"  "minimum RC Angle" "config.minRCRoll: the amount or rotation your motor will make on that axis"
        gui_slider .note.rollRC.set.rc.rcmax  maxRCRoll -50 50 1           "RC max"  "maximum RC Angle" "config.maxRCRoll: the amount or rotation your motor will make on that axis"
        gui_slider .note.rollRC.set.rc.aop angleOffsetRoll -50 50 0.1      "Zero Offset" "Zero Offset" "config.angleOffsetRoll: offset adjust for roll zero position (deg)"

      labelframe .note.rollRC.set.fpv -text "FPV Activation" -padx 10 -pady 10
      pack .note.rollRC.set.fpv -side top -expand no -fill x
        gui_combobox .note.rollRC.set.fpv.fpvSw  fpvSwRoll                "FPV Switch" "select FPV control switch" "config.fpvSwRoll: RC Switch for FPV mode, legal values -1=always on, 0=off, 1=auxSW1, 2=auxSW2"
        gui_combobox .note.rollRC.set.fpv.fpvFreezePitch fpvFreezeRoll    "FPV Mode"   "select FPV mode" "config.fpvFreezePitch: select between theses modes, Follow and Freeze. In Follow Mode the camera position follows the FPV RC channel coming from the flight control. The Freeze Mode can be used for light weight gimbals. During thos mode the control loop is stopped and the motor drive is frozen at the current position"
      
      labelframe .note.rollRC.set.fpvMx -text "FPV Follow Mode Parameters" -padx 10 -pady 10
      pack .note.rollRC.set.fpvMx -side top -expand no -fill x
        gui_combobox  .note.rollRC.set.fpvMx.rcModePPMFpv rcModePPMFpvR        "PPM/PWM" "Mode of RC input, PPM sum oder single PWM RC inputs on A1/A2" "config.rcModePPM: PPM sum oder single PWM RC inputs on A0/A1/A2: PPM sum input on A2 or single RC PWM inputs on A2=Ch0, A1=Ch1, A0=Ch3"
        gui_spin   .note.rollRC.set.fpvMx.rcChannelFpv rcChannelFpvR 0 16 1    "RC Channel #"  "rcChannelFPV" "config.rcChannelFpvRoll: RC channel number for RC Aux Switch auxSW1/auxSW2, legal values 1..16 in PPM mode, 1..3 in PWM mode, 0=OFF (disabled)"
        gui_slider .note.rollRC.set.fpvMx.fpvGain fpvGainRoll -100 100.0 0.1   "FPV gain" "FPV gain" "config.fpvGainRoll: Gain of FPV channel: specifies the gain of the FPV channel, change sign to reverse direction"
        gui_slider .note.rollRC.set.fpvMx.rcLPFRollFpv rcLPFRollFpv 0.1 20 0.1 "FPV Low Pass" "FPV low pass filter" "config.rcLPFRollFpv: RC low pass filter constant(sec)"

      labelframe .note.rollRC.set.fpvFreeze -text "FPV Freeze Mode Parameters" -padx 10 -pady 10
      pack .note.rollRC.set.fpvFreeze -side top -expand no -fill x

        gui_slider .note.rollRC.set.fpvFreeze.maxPWMfpvRoll maxPWMfpvRoll 0 100 0.1 "motorPWM (%)" "alternate motor power in fpv freeze mode" "config.maxPWMfpvRoll: during fpv freeze mode, this power setting is used to increase torque"

    labelframe .note.rollRC.monitor -text "RC Monitor"
    pack .note.rollRC.monitor -side top -expand yes -fill both
      gui_monitor .note.rollRC.monitor.rcRoll rcRoll "RC Roll" "SW FPV" bar
      gui_monitor .note.rollRC.monitor.rcFpvRoll rcFpvRoll "FPV Roll" "SW FPV" bar
    
  ttk::frame .note.cal
  .note add .note.cal -text "Calibration"

    labelframe .note.cal.acc -text "ACC Sensor" -padx 10 -pady 10
    pack .note.cal.acc -side top -expand no -fill x
      gui_spin .note.cal.acc.accOffsetX     accOffsetX  -500 500 1 "Offset X"  "accOffsetX" "config.accOffsetX"
      gui_spin .note.cal.acc.accOffsetY     accOffsetY  -500 500 1 "Offset Y"  "accOffsetY" "config.accOffsetY"
      gui_spin .note.cal.acc.accOffsetZ     accOffsetZ  -500 500 1 "Offset Z"  "accOffsetZ" "config.accOffsetZ"
      gui_button .note.cal.acc.acc_cal "ACC Calibration" "ACC calibration" acc_cal
      gui_button .note.cal.acc.acc_cal_res "Reset" "reset acc calibration" acc_cal_reset

    labelframe .note.cal.gyro -text "GYRO Sensor" -padx 10 -pady 10
    pack .note.cal.gyro -side top -expand no -fill x
      gui_check .note.cal.gyro.gyroCal      gyroCal   "Calibration" "at Startup ON" "gyroCal" "config.gyroCal: enable gyro calibration at startup"
      gui_spin .note.cal.gyro.gyroOffsetX   gyrOffsetX  -500 500 1 "Offset X"  "gyroOffsetX" "config.gyrOffsetX"
      gui_spin .note.cal.gyro.gyroOffsetY   gyrOffsetY  -500 500 1 "Offset Y"  "gyroOffsetY" "config.gyrOffsetY"
      gui_spin .note.cal.gyro.gyroOffsetZ   gyrOffsetZ  -500 500 1 "Offset Z"  "gyroOffsetZ" "config.gyrOffsetZ"
      gui_button .note.cal.gyro.gyro_cal "GYRO Calibration" "gyro calibration" gyro_cal
      gui_button .note.cal.gyro.gyro_cal_res "Reset" "reset gyro calibration" gyro_cal_reset

  ttk::frame .note.aux
  .note add .note.aux -text "Auxiliary"

    labelframe .note.aux.1 -padx 10 -pady 7
    pack .note.aux.1 -side top -expand no -fill x

      labelframe .note.aux.1.rc -text "RC Auxiliary Switch Channel" -padx 10 -pady 7
      pack .note.aux.1.rc -side left -expand yes -fill x
        gui_combobox .note.aux.1.rc.rcModePPMPAux rcModePPMAux         "RC PPM/PWM" "Mode of RC input, PPM sum oder single PWM RC inputs on A1/A2" "config.rcModePPM: PPM sum oder single PWM RC inputs on A0/A1/A2: PPM sum input on A2 or single RC PWM inputs on A2=Ch0, A1=Ch1, A0=Ch3"
        gui_spin     .note.aux.1.rc.rcChannelAux  rcChannelAux 0 16 1  "RC Channel #"  "rcChannelAux" "config.rcChannelAux: RC channel number for RC Aux Switch auxSW1/auxSW2, legal values 1..16 in PPM mode, 1..3 in PWM mode, 0=OFF (disabled)"

      labelframe .note.aux.1.altTC -text "Alternate ACC Time Constant" -padx 10 -pady 7
      pack .note.aux.1.altTC -side left -expand yes -fill x
        gui_combobox   .note.aux.1.altTC.altSwAccTime  altSwAccTime   "ACC Time2 Switch" "select control switch for alternate ACC time" "config.altSwAccTime: RC Switch for alternate ACC time constant, legal values -1=always on, 0=off, 1=auxSW1, 2=auxSW2"
        gui_slider .note.aux.1.altTC.accTimeConstant2  accTimeConstant2 1 20 0.1  "ACC Time2"  "accTimeConstant2" "config.accTimeConstant2: alternate value for ACC Time Constant, activated by wwitch function altSwAccTime"

    labelframe .note.aux.rcpin -text "Mode of Control Inputs" -padx 10 -pady 7
    pack .note.aux.rcpin -side top -expand no -fill x
      gui_combobox  .note.aux.rcpin.rcPinModeCH0 rcPinModeCH0 "Input 1 (A0)" "Set Control Input Mode OFF/Digital/Analog" "config.rcPinModeCH0: Set Control Input OFF/Digital/Analog: 0=Off, 1=RC digital PWM/PPM mode, 2=analog input A2"
      gui_combobox  .note.aux.rcpin.rcPinModeCH1 rcPinModeCH1 "Input 2 (A1)" "Set Control Input Mode OFF/Digital/Analog" "config.rcPinModeCH1: Set Control Input OFF/Digital/Analog: 0=Off, 1=RC digital PWM mode, 2=analog input A1"
      gui_combobox  .note.aux.rcpin.rcPinModeCH2 rcPinModeCH2 "Input 3 (A0)" "Set Control Input Mode OFF/Digital/Analog" "config.rcPinModeCH2: Set Control Input OFF/Digital/Analog: 0=Off, 1=RC digital PWM mode, 2=analog input A0"
      
    labelframe .note.aux.monitor -text "RC Monitor" -padx 10 -pady 7
      pack .note.aux.monitor -side top -expand no -fill x
      gui_monitor .note.aux.monitor.rcAux rcAux "RC Auxiliary" no bar
      gui_monitor .note.aux.monitor.rcAuxSW1 rcAuxSW1 "AuxSW1" no no
      gui_monitor .note.aux.monitor.rcAuxSW2 rcAuxSW2 "AuxSW2" no no
      
    labelframe .note.aux.debug -text "Debug (just for development purposes)" -padx 10 -pady 7
    pack .note.aux.debug -side top -expand no -fill x

      gui_spin .note.aux.debug.sTrace     sTrace    0 9 1 "Trace Mode (slow)"  "sTrace" "config.sTrace"
#      gui_spin .note.aux.debug.fTrace     fTrace    0 9 1 "Trace Mode (fast)"  "fTrace" "config.fTrace"


frame .common -padx 1 -pady 1
pack .common -side top -expand yes -fill y
      
  labelframe .common.monitor -text "Monitor"  -padx 10 -pady 7
  pack .common.monitor -side top -expand no -fill x
  setTooltip .common.monitor "Monitor Functions"

    button .common.monitor.liveViewButton -text "Live View" -width 13 -command {
      live_view
    }
    pack .common.monitor.liveViewButton -side left -expand no -fill x

  frame .common.chartview
  pack .common.chartview -side top -expand no -fill x
    labelframe .common.chartview.chart -text "Waveform"
    pack .common.chartview.chart -side top -expand no -fill both

      canvas .common.chartview.chart.chart1 -relief raised -width 450 -height 100
      pack .common.chartview.chart.chart1 -side top
      .common.chartview.chart.chart1 create rec 1 1 450 100 -fill black
      .common.chartview.chart.chart1 create line 0 50 450 50 -fill white
      setTooltip .common.chartview.chart.chart1 "waveform view"

      frame .common.chartview.chart.fr1
      pack .common.chartview.chart.fr1 -side left -expand yes -fill both

        button .common.chartview.chart.fr1.button -text "Start Waveform View" -width 5 -relief raised  -background lightgrey -command {
                draw_chart
        }
        pack .common.chartview.chart.fr1.button -side top -expand yes -fill both
        setTooltip .common.chartview.chart.fr1.button "start/stop waveform view"

        frame .common.chartview.chart.fr1.scale
        pack .common.chartview.chart.fr1.scale -side top -expand no -fill x

          scale .common.chartview.chart.fr1.scale.slider -orient horizontal -from 0.1 -to 100.0 -showvalue 0 -resolution 0.1 -variable CHART_SCALE
          pack .common.chartview.chart.fr1.scale.slider -side left -expand yes -fill x
          setTooltip .common.chartview.chart.fr1.scale.slider "Y-Scale for waveform view"
          spinbox .common.chartview.chart.fr1.scale.spin -from 0.1 -to 100.0 -increment 0.1 -width 10 -textvariable CHART_SCALE -width 4
          pack .common.chartview.chart.fr1.scale.spin -side left -expand no -fill x

          button .common.chartview.chart.fr1.scale.help -text "?" -width 1 -command {
                show_help "Y-Scale for waveform view"
          }
          pack .common.chartview.chart.fr1.scale.help -side right -expand no -fill none

  labelframe .common.device -text "Connection"
  pack .common.device -side top -expand no -fill x
  setTooltip .common.device "serial port selection"

    label .common.device.label -text "Port" -width 5
    pack .common.device.label -side left -expand no -fill x

    if {[catch {ttk::combobox .common.device.spin -textvariable device -state readonly -values $comports}]} {
            spinbox .common.device.spin -values $comports -width 10  -textvariable device
    }
    pack .common.device.spin -side left -expand yes -fill x

    button .common.device.connect -text "Connect" -width 9 -command {
            connect_serial
    }
    pack .common.device.connect -side left -expand no -fill x

    button .common.device.close -text "Close" -width 9 -command {
            close_serial
    }
    pack .common.device.close -side left -expand no -fill x


  frame .common.bottom
  pack .common.bottom -side bottom -expand no -fill x

    label .common.bottom.bruGiVersion -width 25 -text "Firmware: -----"
    pack .common.bottom.bruGiVersion -side left -expand no -fill x
    setTooltip .common.bottom.bruGiVersion "BruGi Firmware Version"

    label .common.bottom.version -text "Host: $tcl_platform(os)/$tcl_platform(osVersion)"
    pack .common.bottom.version -side left -expand yes -fill x
    setTooltip .common.bottom.version "Host System Version"

  label .common.helptext -relief sunken -text "Tooltips"
  pack .common.helptext -side bottom -expand no -fill x
    
  frame .common.bottom1
  pack .common.bottom1 -side bottom -expand no -fill x

    label .common.bottom1.bruGiStatus -width 12 -text "BruGi"
    pack .common.bottom1.bruGiStatus -side left -expand no -fill x
    setTooltip .common.bottom1.bruGiStatus "BruGi status"

    label .common.bottom1.message -text "no message"
    pack .common.bottom1.message -side left -expand yes -fill x
    setTooltip .common.bottom1.message "BruGi message window"

    label .common.bottom1.linkStatus -width 10 -text "Link"
    pack .common.bottom1.linkStatus -side left -expand no -fill x
    setTooltip .common.bottom1.linkStatus "Link Status"
    
  frame .common.bottom2
  pack .common.bottom2 -side bottom -expand no -fill x

    label .common.bottom2.rxlog -relief sunken -text "" -width 45
    pack .common.bottom2.rxlog -side left -expand no -fill x
    setTooltip .common.bottom2.rxlog "receive messages from BruGi"

    label .common.bottom2.txlog -relief sunken -text "" -width 30
    pack .common.bottom2.txlog -side right -expand no -fill x
    setTooltip .common.bottom2.txlog "receive messages from BruGi"
    

# GUI end
##################################################################################### 
  
  
## Trace parameters to update on change
foreach var [array names par] {
  if {! [string match "*,*" $var]} {
    trace variable par($var) w send_parvar
  }
}

## Trace parameters to update on change
foreach var [array names traceVar] {
  trace variable traceVar($var) w update_traceDisplay
}


# call display update for all trace values
proc init_traceDisplay {} {
  global traceVar
  global liveViewON
  set liveViewON 0
  foreach var [array names traceVar] {
    update_traceDisplay traceVar($var) $var  w
  }
}

# update display of traced variables
proc update_traceDisplay {n1 n2 op} {
  global traceVar
  global liveViewON

  if {$liveViewON == 1} {
    set bg_color_act "green"
    set bg_color_inact "yellow"
    set bg_color_warn "red"
  } else {
    set bg_color_act "lightgrey"
    set bg_color_inact "lightgrey"
    set bg_color_warn "lightgrey"  }

  # Pitch
  if {$n2 == "rcPitch"} {
    gui_monitor_update_bar .note.pitchRC.monitor.rcPitch $traceVar($n2) 900.0 2100.0
  }
  if {$n2 == "rcPitchValid"} {
    if {$traceVar($n2) == 1} {
       gui_monitor_update_bar_color .note.pitchRC.monitor.rcPitch "$bg_color_act"
    } else {
       gui_monitor_update_bar_color .note.pitchRC.monitor.rcPitch "$bg_color_inact"
    }
  }
  if {$n2 == "rcFpvPitch"} {
    gui_monitor_update_bar .note.pitchRC.monitor.rcFpvPitch $traceVar($n2) 900.0 2100.0
  }
  if {$n2 == "rcFpvPitchValid"} {
    if {$traceVar($n2) == 1} {
      gui_monitor_update_bar_color .note.pitchRC.monitor.rcFpvPitch "$bg_color_act"
    } else {
      gui_monitor_update_bar_color .note.pitchRC.monitor.rcFpvPitch "$bg_color_inact"
    }
  }
  if {$n2 == "fpvModePitch"} {
    if {$traceVar($n2) == 1} {
       gui_monitor_update_color .note.pitchRC.monitor.rcPitch.sw lightgrey
       gui_monitor_update_color .note.pitchRC.monitor.rcFpvPitch.sw "$bg_color_act"
    } else {
       gui_monitor_update_color .note.pitchRC.monitor.rcPitch.sw "$bg_color_act"
       gui_monitor_update_color .note.pitchRC.monitor.rcFpvPitch.sw lightgrey
    }
  }
  
  # Roll
  if {$n2 == "rcRoll"} {
    gui_monitor_update_bar .note.rollRC.monitor.rcRoll $traceVar($n2) 900.0 2100.0
  }
  if {$n2 == "rcRollValid"} {
    if {$traceVar($n2) == 1} {
       gui_monitor_update_bar_color .note.rollRC.monitor.rcRoll "$bg_color_act"
    } else {
       gui_monitor_update_bar_color .note.rollRC.monitor.rcRoll "$bg_color_inact"
    }
  }
  if {$n2 == "rcFpvRoll"} {
    gui_monitor_update_bar .note.rollRC.monitor.rcFpvRoll $traceVar($n2) 900.0 2100.0
  }
  if {$n2 == "rcFpvRollValid"} {
    if {$traceVar($n2) == 1} {
      gui_monitor_update_bar_color .note.rollRC.monitor.rcFpvRoll "$bg_color_act"
    } else {
      gui_monitor_update_bar_color .note.rollRC.monitor.rcFpvRoll "$bg_color_inact"
    }
  }
  if {$n2 == "fpvModeRoll"} {
    if {$traceVar($n2) == 1} {
       gui_monitor_update_color .note.rollRC.monitor.rcRoll.sw lightgrey
       gui_monitor_update_color .note.rollRC.monitor.rcFpvRoll.sw "$bg_color_act"
    } else {
       gui_monitor_update_color .note.rollRC.monitor.rcRoll.sw "$bg_color_act"
       gui_monitor_update_color .note.rollRC.monitor.rcFpvRoll.sw lightgrey
    }
  }

  # Aux
  if {$n2 == "rcAux"} {
    gui_monitor_update_bar .note.aux.monitor.rcAux $traceVar($n2) 900.0 2100.0
  }
  if {$n2 == "rcAuxValid"} {
    if {$traceVar($n2) == 1} {
       gui_monitor_update_bar_color .note.aux.monitor.rcAux "$bg_color_act"
    } else {
       gui_monitor_update_bar_color .note.aux.monitor.rcAux "$bg_color_inact"
    }
  }
  if {$n2 == "rcAuxSW1"} {
    gui_monitor_update .note.aux.monitor.rcAuxSW1 $traceVar($n2)
    if {$traceVar($n2) == 0} {
      gui_monitor_update_color .note.aux.monitor.rcAuxSW1.value "$bg_color_inact"
    } else {
      gui_monitor_update_color .note.aux.monitor.rcAuxSW1.value "$bg_color_act"
    }
  }
  if {$n2 == "rcAuxSW2"} {
    gui_monitor_update .note.aux.monitor.rcAuxSW2 $traceVar($n2)
    if {$traceVar($n2) == 0} {
      gui_monitor_update_color .note.aux.monitor.rcAuxSW2.value "$bg_color_inact"
    } else {
      gui_monitor_update_color .note.aux.monitor.rcAuxSW2.value "$bg_color_act"
    }
  }

  # MPU
  if {$n2 == "mpuI2cErrors"} {
    gui_monitor_update .note.general.settings.sensor.set.mpuMonitor.mpuI2cErrors $traceVar($n2)
  }
  if {$n2 == "mpuI2cErrors"} {
    if {$traceVar($n2) == 0} {
      gui_monitor_update_color .note.general.settings.sensor.set.mpuMonitor.mpuI2cErrors.value "$bg_color_act"
    } else {
      gui_monitor_update_color .note.general.settings.sensor.set.mpuMonitor.mpuI2cErrors.value "$bg_color_warn"
    }
  }
  
  if {$n2 == "mpuTemp"} {
    gui_monitor_update .note.general.settings.sensor.set.mpuMonitor.mpuTemp $traceVar($n2)
    gui_monitor_update_color .note.general.settings.sensor.set.mpuMonitor.mpuTemp.value "$bg_color_act"
  }
  
}
    

#####################################################################################
# check arguments
#####################################################################################

if {[lindex $argv 0] == "-b" || [lindex $argv 0] == "--help"} {
        if {[lindex $argv 1] == ""} {
                puts "[info script] -b firmware.bin"
                exit 0
        }
        set FILE [lindex $argv 1]

        if {[string match "*.hex" $FILE]} {
                exec objcopy -I ihex -O binary "$FILE" "$FILE.bin"
                set FILE "$FILE.bin"
        }

        set scriptfile [open "[info script]" r]
        set scriptdata [read $scriptfile]
        close $scriptfile

        set scriptfile [open "[info script].new" w]
        foreach line "[split $scriptdata "\n"]" {
                if {[string match "set HEXDATA *" $line]} {
                        puts $scriptfile "set HEXDATA \"[ArduinoExportBIN $FILE]\""
                } else {
                        puts $scriptfile "$line"
                }
        }
        close $scriptfile
        exit 0
}


proc show_help_about {} {
        catch {destroy .help}
        toplevel .help
        wm title .help "About"
        frame .help.f -highlightthickness 1 -borderwidth 1 -relief sunken
        pack .help.f -expand yes -fill both
        text .help.f.t -yscrollcommand ".help.f.scroll set" -setgrid true -width 80 -height 20 -wrap word -highlightthickness 0 -borderwidth 0
        pack .help.f.t -side left -expand  yes -fill both
        scrollbar .help.f.scroll -command ".help.f.t yview"
        pack .help.f.scroll -side right -fill y
        .help.f.t tag configure header -font "times 24 bold" -justify center
        .help.f.t tag configure center -justify center -spacing1 10p -spacing2 2p -lmargin1 12m -lmargin2 6m -rmargin 10m
        .help.f.t tag configure text -spacing1 10p -spacing2 2p -lmargin1 12m -lmargin2 6m -rmargin 10m
        .help.f.t insert end "\n" text
        .help.f.t insert end "Credits\n" center
        .help.f.t insert end "Manual: By Graham Miller\n" center
        .help.f.t insert end "Brushless Gimbal: By Ludwig Faerber\n" center
        .help.f.t insert end "GUI: By Oliver Dippel\n" center
        .help.f.t insert end "Software: By\n" center
        .help.f.t insert end "Christian Winkler , Ludwig Faerber, Alois Hahn and Alexander Rehfeld\n" center
        .help.f.t insert end "All rights reserved\n" center

        button .help.exit -text "Exit" -width 14 -command {
                destroy .help
        }
        pack .help.exit -side bottom -expand no -fill x
}