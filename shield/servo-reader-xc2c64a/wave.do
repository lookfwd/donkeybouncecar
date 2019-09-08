onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /servocount_tb/clk
add wave -noupdate /servocount_tb/in0
add wave -noupdate /servocount_tb/in1
add wave -noupdate /servocount_tb/scan_en
add wave -noupdate /servocount_tb/scan_in
add wave -noupdate /servocount_tb/scan_out
add wave -noupdate -divider {New Divider}
add wave -noupdate /servocount_tb/uut/s0/state
add wave -noupdate -radix unsigned /servocount_tb/uut/s0/count
add wave -noupdate -radix unsigned /servocount_tb/uut/s0/long_sum
add wave -noupdate -radix unsigned /servocount_tb/uut/s0/next_count
add wave -noupdate /servocount_tb/uut/s0/overflow
add wave -noupdate -divider {New Divider}
add wave -noupdate /servocount_tb/uut/s1/state
add wave -noupdate -radix unsigned /servocount_tb/uut/s1/count
add wave -noupdate -radix unsigned /servocount_tb/uut/s1/long_sum
add wave -noupdate -radix unsigned /servocount_tb/uut/s1/next_count
add wave -noupdate /servocount_tb/uut/s1/overflow
add wave -noupdate -divider {New Divider}
add wave -noupdate -radix binary /servocount_tb/uut/s0/count
add wave -noupdate /servocount_tb/uut/s0/state
add wave -noupdate -radix binary /servocount_tb/uut/s1/count
add wave -noupdate /servocount_tb/uut/s1/state
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {4080171403 ps} 0}
configure wave -namecolwidth 229
configure wave -valuecolwidth 83
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ps
update
WaveRestoreZoom {3975563536 ps} {4103436464 ps}
