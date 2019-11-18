#!/usr/bin/env python3
f = open("/home/jspark/logfiles/logdata1.txt",'w')
for i in range(1, 11):
    data = "%d %d\n" % (i, i*i)
    f.write(data)
f.close()


ff = open("/home/jspark/logfiles/logdata1.txt", 'r')
data = ff.read()
print(data)
ff.close()
