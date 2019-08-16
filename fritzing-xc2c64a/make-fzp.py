pins = [
        "IO_22",
        "IO_23",
        "IO_24",
        "GND",
        "IO_25",
        "IO_26",
        "VDDIO1",
        "IO_27",
        "TDI",
        "TMS",
        "TCK",

        "IO_28",
        "IO_29",
        "IO_30",
        "VDDINT",
        "IO_31",
        "GND",
        "IO_32",
        "IO_15",
        "IO_14",
        "IO_13",
        "IO_12",

        "IO_11",
        "TDO",
        "GND",
        "VDDIO2",
        "IO_10",
        "IO_9",
        "IO_8",
        "IO_7",
        "IO_6",
        "IO_5",
        "IO_4",

        "IO_3",
        "VDDAUX",
        "IO_2",
        "IO_1",
        "IO_0",
        "IO_16",
        "IO_17",
        "IO_18",
        "IO_19",
        "IO_20",
        "IO_21",
]

for i, t in enumerate(pins):
    print(f'''  <connector type="male" name="{t}" id="connector{i}">
   <description>{t}</description>
   <views>
    <breadboardView>
     <p svgId="connector{i}pin" layer="breadboard"/>
    </breadboardView>
    <schematicView>
     <p svgId="connector{i}pin"  terminalId="connector{i}terminal" layer="schematic"/>
    </schematicView>
    <pcbView>
     <p svgId="connector{i}pad" layer="copper1"/>
    </pcbView>
   </views>
  </connector>''')
