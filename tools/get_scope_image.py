import vxi11

instr = vxi11.Instrument('10.10.255.228')
instr.open()
print(instr.ask('*IDN?'))  # sanity check

# Screenshot
data = instr.ask_raw(b'SCDP\n')
with open('scope.bmp', 'wb') as f:
    f.write(data)
instr.close()
