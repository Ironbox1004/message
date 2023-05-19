## 事件检测相关代码，主要包含有：

### 1.人流量统计
```angular2html
通过对configs.py脚本中的Pperson_sort_list值进行修改，来自定义不同需要统计的人流量信息
```
### 2.车流量统计

```angular2html
通过对configs.py脚本中的Vehicle_sort_list值进行修改，来自定义不同需要统计的车道信息
```
### 3.人员进入危险区域预警
```angular2html
通过对configs.py脚本中的danger_area值进行修改，来自定义不同需要统计的危险区域信息
```
### 4.车辆排队数量与车辆排队长度统计
```angular2html
通过对configs.py脚本中的Vehicle_area_list值进行修改，来自定义不同需要统计的排队区域信息,
排队长度通过Vehicle_line_head定义每条车道的起始位置，通过计算出最后一辆车的位置与起始定义
位置之间的距离作为排队长度
```
### 5.车辆拥堵判定
```angular2html
目前可以分为：CLEAR = 0 # 畅通； BASIC_CLEAR = 1 # 基本畅通
LIGHT = 2  # 轻度拥堵； MODERATE = 3  # 中度拥堵
SERIOUS = 4  # 严重拥堵； INVALID = 5  # 无效值
```
### 6.车辆逆行判定
```angular2html
通过计算角度ROAD_DIR的反向角度ra
```

### 日志管理脚本
```
通过nb_log库进行日志管理，安装方式如下：
pip install nb_log
```
参考链接: [nb_log安装使用](https://github.com/ydf0509/nb_log)