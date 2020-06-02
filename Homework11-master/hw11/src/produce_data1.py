import numpy as np
import pylab as plt
import csv

time_stamp = []
num_list = []
receve_list= []

for t in range(0,46,1):
    d = (2.0 * t) + ( t**2.0 )
    num_list.append(d)
    receve_list.append(d)
    time_stamp.append(t)

## 製造數據
for i in range(0,46,1):
    receve_list[i] = round(num_list[i] + np.random.normal(0, 20),5)
print(num_list)
print(receve_list)

# 開啟 CSV 檔案
with open('/home/ee405423/Desktop/output.csv', 'w', newline='') as csvfile:
# 建立 CSV 檔寫入器
  writer = csv.writer(csvfile)
  # 寫入一列資料
  writer.writerow(num_list)
  writer.writerow(receve_list)



plt.figure(figsize=(6,8))
plt.subplot(411)
plt.title('The Original Signal')
plt.plot(time_stamp[0:46],num_list[0:46])

plt.title('Noise Signal')
plt.subplot(412)
plt.plot(time_stamp[0:46],receve_list[0:46])

plt.show()
plt.tight_layout()
