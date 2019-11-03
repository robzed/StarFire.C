import sys

class WrapClassU16:
    def __init__(self, initial_value):
        self.old = initial_value
        self.current = 0

    def update(self, new_value):
        if new_value < self.old:
            # wrapped
            new_value += 65536
        self.current += new_value - self.old
        self.old = new_value

    def value(self):
        return self.current

f = open("starfire_calexport2.txt")
data = f.readlines()
f.close

# first line is number of entries
count = int(data[0])
#print("Number of entries", count)
#print("Number of lines", len(data))
#print(data[1])
data = data[1:]
#print(data[0])

data_table = []
for line in data:
    s = line.strip()
    if s != "":
        s = s.split()
        if len(s) != 7:
            print("Expected 7 elements")
            sys.exit(1)

        data_int = []
        for element in s:
            data_int.append(int(element))
        data_table.append(data_int)

if len(data_table) != count:
    print("Warning - data contained", len(data_table), "entries, but file header said", count, "entries")

# display the data
print("tick, left wheel count, right wheel count, right front sensor, left diagonal sensor, right diagonal sensor, left front sensor")
tick = WrapClassU16(data_table[0][0])
lcount = WrapClassU16(data_table[0][1])
rcount = WrapClassU16(data_table[0][2])

for line in data_table:
    tick.update(line[0])
    lcount.update(line[1])
    rcount.update(line[2])
    r_front = line[3]
    l_diag = line[4]
    r_diag = line[5]
    l_front = line[6]

    print("%.4f, %d, %d, %d, %d, %d, %d" % (tick.value()/10000, lcount.value(), rcount.value(), r_front, l_diag, r_diag, l_front))

