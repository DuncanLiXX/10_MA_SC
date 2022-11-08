# SC报警文档

## 报警级别

| 报警级别    | 说明            |
| ------- | ------------- |
| 严重错误（0） | 停止加工，下伺服，报警显示 |
| 错误（1）   | 停止加工，报警显示     |
| 警告（2）   | 报警显示          |
| 提示（3）   | 报警显示          |

## 报警类型

| 报警类型      | 说明                    |
| --------- | --------------------- |
| 断电重启清除（0） | 持续保留，断电后清除            |
| 复位清除（1）   | 持续保留，复位后清除            |
| 自动清除（2）   | 梯图报警，条件满足报警，条件不满足自动清除 |

## 报警内容

### sc内部错误 0-999

| 报警编号 | 报警信息              | 报警级别    | 报警类型    |
| ---- | ----------------- | ------- | ------- |
| 1    | SC模块初始化出错         | 错误（1）   | 断电重启（0） |
| 2    | MI通讯模块初始化出错       | 严重错误(0) | 断电重启（0） |
| 3    | MC通讯模块初始化出错       | 严重错误(0) | 断电重启（0） |
| 4    | 内存分配失败            | 错误（1）   | 断电重启（0） |
| 5    | 打开文件目录失败          | 警告（2）   | 复位清除（1） |
| 6    | 打开文件失败            | 错误（1）   | 复位清除（1） |
| 7    | 读取NC文件失败          | 错误（1）   | 复位清除（1） |
| 8    | 文件另存为失败           | 警告（2）   | 复位清除（1） |
| 9    | 加载MDA数据失败         | 错误（1）   | 复位清除（1） |
| 10   | 初始化断点继续线程失败       | 错误（1）   | 断电重启（0） |
| 11   | 初始化模块升级线程失败       | 错误（1）   | 断电重启（0） |
| 12   | 初始化加载SPARTAN6代码失败 |         |         |
| 13   | 初始化加载MI模块失败       |         |         |
| 14   | MC启动失败            | 严重错误(0) | 断电重启（0） |
| 15   | MI启动失败            | 严重错误(0) | 断电重启（0） |
| 16   | 掉电告警              |         |         |
| 17   | 配置导入后请先重启         | 警告（2）   | 复位清除（1） |
| 18   | 通道主轴数量超限          | 错误（1）   | 复位清除（1） |
| 19   | SDLINK配置信息错误      | 错误（1）   | 断电重启（0） |
|      |                   |         |         |
| 82   | 无授权               | 错误（1）   | 复位清除（1） |
| 83   | 非法授权              | 错误（1）   | 复位清除（1） |
| 84   | 系统授权已过期           | 错误（1）   | 复位清除（1） |
| 85   | 系统授权即将到期          | 提示（3）   | 复位清除（1） |
|      |                   |         |         |
| 107  | 螺补数据导入失败          | 错误（1）   | 复位清除（1） |
| 108  | 一键升级失败            |         |         |
|      |                   |         |         |
| 150  | 同步轴建立同步失败         | 错误（1）   | 复位清除（1） |
| 151  | 同步轴解除同步失败         | 错误（1）   | 复位清除（1） |
| 152  | 同步轴初始化同步失败        | 错误（1）   | 复位清除（1） |
|      |                   |         |         |
| 500  | 过压告警              | 警告（2）   | 复位清除（1） |
| 501  | 欠压告警              | 警告（2）   | 复位清除（1） |
| 502  | 电池电压低             | 警告（2）   | 复位清除（1） |
| 503  | 轴过流告警             | 警告（2）   | 复位清除（1） |
| 504  | USB过流告警           | 警告（2）   | 复位清除（1） |
|      |                   |         |         |
| 600  | 回参考点失败            | 警告（2）   | 复位清除（1） |
| 601  | 轴未回参考点            | 警告（2）   | 复位清除（1） |
| 602  | 系统处于错误状态，禁止回零     |         |         |
| 603  | 文件不存在             | 警告（2）   | 复位清除（1） |
| 604  | 回零中禁止切换工作模式       | 警告（2）   | 复位清除（1） |
| 605  | 搜索Z脉冲位置超限，原点建立失败  | 警告（2）   | 复位清除（1） |
|      |                   |         |         |

### 加工警告 1000-1999

| 报警编号 |           报警信息           | 报警级别  |   报警类型    |
| ------- | --------------------------- | -------- | ------------ |
| 1000    | 紧急停止                     | 错误（1） | 复位清除（1） |
| 1001    | 轴正限位告警                 | 错误（1） | 复位清除（1） |
| 1002    | 轴负限位告警                 | 错误（1） | 复位清除（1） |
| 1003    | 编码器错误                   | 错误（1） | 复位清除（1） |
| 1004    | 伺服告警                     | 错误（1） | 复位清除（1） |
| 1005    | 同步轴尚未同步               | 错误（1） | 复位清除（1） |
| 1006    | 轴正向软限位告警              | 警告（2） | 复位清除（1） |
| 1007    | 轴负向软限位告警              | 警告（2） | 复位清除（1） |
| 1008    | 轴跟随误差过大告警            | 错误（1） | 复位清除（1） |
| 1009    | 同步轴位置指令偏差过大告警     | 错误（1） | 复位清除（1） |
| 1010    | 轴位置指令过大告警            | 错误（1） | 复位清除（1） |
| 1011    | 轴控制模式切换超时            | 错误（1） | 复位清除（1） |
| 1012    | 轴力矩控制超时               | 错误（1） | 复位清除（1） |
| 1013    | 不支持的M指令                | 错误（1） | 复位清除（1） |
| 1014    | 主轴预启动期间不能调用M05     |          |              |
| 1015    | 系统未指定主轴或Z轴           | 错误（1） | 复位清除（1） |
|         |                             |          |              |
| 1500    | 找不到当前运行数据            | 错误（1） | 复位清除（1） |
|         |                             |          |              |
| 1650    | 手轮反向跟踪无更多数据        | 警告（2） | 复位清除（1） |
| 1651    | 刀具寿命已完                 |          |              |
| 1652    | 刀具寿命即将到达              | 错误（1） | 复位清除（1） |
| 1700    | 刚性攻丝失败(没有进入位置模式) | 错误（1） | 复位清除（1） |
| 1701    | 刚攻状态发送速度指令          | 错误（1） | 复位清除（1） |
| 1702    | 刚性攻丝比例异常              | 错误（1） | 复位清除（1） |
| 1703    | 定位时主轴转速异常            | 错误（1） | 复位清除（1） |
| 1704    | 主轴换挡超时                 | 错误（1） | 复位清除（1） |
| 1705    | 刚攻位置错误                 | 错误（1） | 复位清除（1） |
| 1706    | 刚性攻丝中不能复位            | 错误（1） | 复位清除（1） |
| 1707    | 刚性攻丝回退无效              | 错误（1） | 复位清除（1） |
| 1708    | 刚性攻丝回退失败              | 错误（1） | 复位清除（1） |
| 1709    | 自动模式禁止攻丝回退          | 错误（1） | 复位清除（1） |
|         |                             |          |              |
|         |                             |          |              |
| 1900    | PMC轴控制状态切换            | 错误（1） | 复位清除（1） |

### 编译警告 2000-3999

| 报警编号 | 报警信息                   | 报警级别  | 报警类型    |
| ---- | ---------------------- | ----- | ------- |
| 2000 | 编译器内部错误                | 错误（1） | 断电重启（0） |
| 2001 | 无%开头                   | 错误（1） | 复位清除（1） |
| 2002 | 无结束命令                  | 错误（1） | 复位清除（1） |
| 2003 | 无法解析                   | 错误（1） | 复位清除（1） |
| 2004 | 无法解析的宏指令格式             | 错误（1） | 复位清除（1） |
| 2005 | 无法解析的宏运算符              | 错误（1） | 复位清除（1） |
| 2006 | 非法代码，不支持的代码            | 错误（1） | 复位清除（1） |
| 2007 | 单行NC代码中G指令个数超限         | 错误（1） | 复位清除（1） |
| 2008 | 单行NC代码中M指令个数超限         | 错误（1） | 复位清除（1） |
| 2009 | 单行NC代码中宏表达式个数超限        | 错误（1） | 复位清除（1） |
| 2010 | 括号不匹配                  | 错误（1） | 复位清除（1） |
| 2011 | 非法的宏表达式                | 错误（1） | 复位清除（1） |
| 2012 | 非法的宏函数操作数              | 错误（1） | 复位清除（1） |
| 2013 | 宏表达式计算错误               | 错误（1） | 复位清除（1） |
| 2014 | 除0错误                   | 错误（1） | 复位清除（1） |
| 2015 | DO指令识别号超范围             | 错误（1） | 复位清除（1） |
| 2016 | DO-END指令不匹配            | 错误（1） | 复位清除（1） |
| 2017 | 存在不可同行的指令              | 错误（1） | 复位清除（1） |
| 2018 | 圆弧指令缺少参数               | 错误（1） | 复位清除（1） |
| 2019 | 圆弧数据错误，无法自洽            | 错误（1） | 复位清除（1） |
| 2020 | 缺少D值指定                 | 错误（1） | 复位清除（1） |
| 2021 | 缺少H值指定                 | 错误（1） | 复位清除（1） |
| 2022 | G01、G02、G03缺少F值指定      | 错误（1） | 复位清除（1） |
| 2023 | 子程序返回，恢复现场异常           | 错误（1） | 复位清除（1） |
| 2024 | 主程序结束异常，子程序调用堆栈非空      | 错误（1） | 复位清除（1） |
| 2025 | 错误的子程序结束指令，子程序必须以M99结束 | 错误（1） | 复位清除（1） |
| 2026 | MDA程序结束后，调用堆栈为空        | 错误（1） | 复位清除（1） |
| 2027 | 无P字段指定子程序号             | 错误（1） | 复位清除（1） |
| 2028 | 存在不能同行的M指令             | 错误（1） | 复位清除（1） |
| 2029 | 预扫描执行失败                | 错误（1） | 断电重启（0） |
| 2030 | 取消预扫描失败                | 错误（1） | 断电重启（0） |
| 2031 | 子程序同名                  | 错误（1） | 复位清除（1） |
| 2031 | 重复的顺序号                 | 错误（1） | 复位清除（1） |
| 2033 | 找不到指定的子程序              | 错误（1） | 复位清除（1） |
| 2034 | 跳转子程序失败                | 错误（1） | 复位清除（1） |
| 2035 | 子程序嵌套层数过多              | 错误（1） | 复位清除（1） |
| 2036 | 没有找到跳转点位置              | 错误（1） | 复位清除（1） |
| 2037 | 执行GOTO指令，跳转失败          | 错误（1） | 复位清除（1） |
| 2038 | END指令执行失败              | 错误（1） | 复位清除（1） |
| 2039 | 指令缺少参数                 | 错误（1） | 复位清除（1） |
| 2040 | 非法跳转                   | 错误（1） | 复位清除（1） |
| 2041 | 缺少P值指定                 | 错误（1） | 复位清除（1） |
| 2042 | 非法L值指定                 | 错误（1） | 复位清除（1） |
| 2043 | 单行NC代码中T指令个数超限         | 错误（1） | 复位清除（1） |
| 2044 | 超长词语，无法解析              | 错误（1） | 复位清除（1） |
| 2045 | 超长数值，无法解析              | 错误（1） | 复位清除（1） |
| 2046 | G指令表达式为空值              | 错误（1） | 复位清除（1） |
| 2047 | M指令表达式为空值              | 错误（1） | 复位清除（1） |
| 2048 | T指令表达式为空值              | 错误（1） | 复位清除（1） |
| 2049 | if else 匹配失败           | 错误（1） | 复位清除（1） |

### 刀补警告 5000-6999

| 报警编号 | 报警信息                         | 报警级别  | 报警类型    |
| ---- | ---------------------------- | ----- | ------- |
| 5000 | 取消刀具半径补偿时没有轴移动               |       |         |
| 5001 | 刀具半径补偿非加工数据超限                |       |         |
| 5002 | C类刀具长度补偿轴个数超限,针对任意指定轴向进行长度补偿 |       |         |
| 5003 | 刀具半径过大                       | 错误(1) | 复位清除（1） |
| 5004 | 刀具半径补偿过程中不可改变加工平面            |       |         |
| 5005 | 建立或取消刀具半径补偿指令错误              | 错误(1) | 复位清除（1） |
| 5006 | 程序结束时没有取消刀具长度补偿              |       |         |
| 5007 | 在机床坐标系下不能使用刀具半径补偿            |       |         |
| 5008 | 刀具长度补偿变化时指定插补指令错误            |       |         |
| 5009 | 不支持指定曲线类型的刀具半径补偿             |       |         |
| 5010 | 刀具半径补偿不能在建立后立即改变             |       |         |
| 5011 | 固定循环中不能使用刀具半径补偿              |       |         |
| 5012 | 刀补轨迹生成失败                     | 错误(1) |         |

### 通讯警告 7000~7999

| 报警编号 | 报警信息         | 报警级别  | 报警类型    |
| ---- | ------------ | ----- | ------- |
| 7000 | HMI心跳丢失      | 错误（1） | 复位清除（1） |
| 7001 | MC模块心跳丢失     | 错误（1） | 断电重启（0） |
| 7002 | MI模块心跳丢失     | 错误（1） | 断电重启（0） |
| 7003 | HMI监控tcp连接中断 | 错误（1） | 复位清除（1） |
| 7004 | 文件传输失败       | 错误（1） | 复位清除（1） |
| 7005 | MC命令包CRC校验错  | 错误（1） | 复位清除（1） |
| 7006 | MI命令包CRC校验错  | 错误（1） | 复位清除（1） |

### MC模块告警 10000~19999

| 报警编号  | 报警信息     | 报警级别  | 报警类型    |
| ----- | -------- | ----- | ------- |
| 10000 | 负向软限位告警  | 错误（1） | 复位清除（1） |
| 10001 | 正向软限位告警  | 错误（1） | 复位清除（1） |
| 10002 | 位置指令过大告警 | 错误（1） | 复位清除（1） |
| 10003 | 圆弧数据错误告警 | 错误（1） | 复位清除（1） |
| 10004 | 指令校验错    | 错误（1） | 复位清除（1） |
| 10005 | 数据校验错    | 错误（1） | 复位清除（1） |

### PMC 告警 20000~20220

| 报警编号        | 报警信息 | 报警级别    | 报警类型    |
| ----------- | ---- | ------- | ------- |
| 20185-20220 |      | 严重错误(0) | 自动清除（2） |
| 20145-20184 |      | 错误（1）   | 自动清除（2） |
| 20081-20144 |      | 警告（2）   | 自动清除（2） |
| 20000-20080 |      | 提示（3）   | 自动清除（2） |

### 总线告警 30000~39999

## XML文档

```xml
<?xml version="1.0" encoding="UTF-8"?>
<root>
    <Item id="1">
        <Message>SC模块初始化出错</Message>
    </Item>
    <Item id="2">
        <Message>MI通讯模块初始化出错</Message>
    </Item>
    <Item id="3">
        <Message>MC通讯模块初始化出错</Message>
    </Item>
    <Item id="4">
        <Message>内存分配失败</Message>
    </Item>
    <Item id="5">
        <Message>打开文件目录失败</Message>
    </Item>        
    <Item id="6">
        <Message>打开文件失败</Message>
    </Item>        
    <Item id="7">
        <Message>读取NC文件失败</Message>
    </Item>        
    <Item id="8">
        <Message>文件另存为失败</Message>
    </Item>        
    <Item id="9">
        <Message>加载MDA数据失败</Message>
    </Item>    
    <Item id="10">
        <Message>初始化断点继续线程失败</Message>
    </Item>
    <Item id="11">
        <Message>初始化模块升级线程失败</Message>
    </Item>
    <Item id="12">
        <Message>初始化加载SPARTAN6代码失败</Message>
    </Item>    
    <Item id="13">
        <Message>初始化加载MI模块失败</Message>
    </Item>    
    <Item id="14">
        <Message>MC启动失败</Message>
    </Item>    
    <Item id="15">
        <Message>MI启动失败</Message>
    </Item>    
    <Item id="16">
        <Message>系统掉电告警</Message>
    </Item>    
    <Item id="17">
        <Message>配置导入后请先重启系统</Message>
    </Item>
    <Item id="18">
        <Message>通道主轴数量超限</Message>
    </Item>
    <Item id="80">
        <Message>系统时间异常</Message>
    </Item>
    <Item id="81">
        <Message>系统文件异常</Message>
    </Item>
    <Item id="82">
        <Message>系统未检测到授权</Message>
    </Item>
    <Item id="83">
        <Message>非法授权</Message>
    </Item>
    <Item id="84">
        <Message>系统授权已过期</Message>
    </Item>
    <Item id="85">
        <Message>系统授权即将到期</Message>
    </Item>
    <Item id="100">
        <Message>MC模块升级失败</Message>
    </Item>        
    <Item id="101">
        <Message>MI模块升级失败</Message>
    </Item>    
    <Item id="102">
        <Message>PL模块升级失败</Message>
    </Item>    
    <Item id="103">
        <Message>SC模块升级失败</Message>
    </Item>    
    <Item id="104">
        <Message>PMC升级失败</Message>
    </Item>    
    <Item id="105">
        <Message>SPARTAN升级失败</Message>
    </Item>
    <Item id="106">
        <Message>Modbus模块升级失败</Message>
    </Item>
    <Item id="107">
        <Message>螺补数据导入失败</Message>
    </Item>
    <Item id="500">
        <Message>过压告警</Message>
    </Item>        
    <Item id="501">
        <Message>欠压告警</Message>
    </Item>    
    <Item id="502">
        <Message>电池电压低</Message>
    </Item>    
    <Item id="503">
        <Message>轴过流告警</Message>
    </Item>    
    <Item id="504">
        <Message>USB过流告警</Message>
    </Item>    
  <Item id="600">
        <Message>回参考点失败</Message>
    </Item>
  <Item id="601">
        <Message>轴未回参考点</Message>
    </Item>
    <Item id="602">
        <Message>系统处于错误状态，禁止回零 </Message>
    </Item>
    <Item id="603">
        <Message>当前文件不存在</Message>
    </Item>
    <Item id="1000">
        <Message>紧急停止</Message>
    </Item>        
    <Item id="1001">
        <Message>轴正向硬限位告警</Message>
    </Item>    
    <Item id="1002">
        <Message>轴负向硬限位告警</Message>
    </Item>    
    <Item id="1003">
        <Message>编码器错误</Message>
    </Item>    
    <Item id="1004">
        <Message>伺服告警</Message>
    </Item>    
    <Item id="1005">
        <Message>同步轴尚未同步</Message>
    </Item>    
    <Item id="1006">
        <Message>轴正向软限位告警</Message>
    </Item>    
    <Item id="1007">
        <Message>轴负向软限位告警</Message>
    </Item>    
    <Item id="1008">
        <Message>轴跟随误差过大告警</Message>
    </Item>    
    <Item id="1009">
        <Message>同步轴位置指令偏差过大告警</Message>
    </Item>    
    <Item id="1010">
        <Message>轴位置指令过大告警</Message>
    </Item>
    <Item id="1011">
        <Message>轴控制模式切换超时 </Message>
    </Item>
    <Item id="1012">
        <Message>轴力矩控制超时 </Message>
    </Item>
    <Item id="1013">
        <Message>不支持的M指令</Message>
    </Item>
    <Item id="1014">
        <Message>主轴预启动期间不能调用M05</Message>
    </Item>
    <Item id="1500">
        <Message>找不到当前运行数据</Message>
    </Item>    
    <Item id="1600">
        <Message>对刀位置未设置</Message>
    </Item>    
    <Item id="1601">
        <Message>自动对刀失败</Message>
    </Item>
    <Item id="1650">
        <Message>反向跟踪无更多数据</Message>
    </Item>
    <Item id="1660">
        <Message>刀具寿命到达</Message>
    </Item>    
    <Item id="1661">
        <Message>刀具寿命即将到达</Message>
    </Item>        
    <Item id="1900">
        <Message>PMC轴控制状态切换</Message>
    </Item>    
    <Item id="2000">
        <Message>编译器内部错误</Message>
    </Item>    
    <Item id="2001">
        <Message>无%开头</Message>
    </Item>    
    <Item id="2002">
        <Message>无结束命令</Message>
    </Item>    
    <Item id="2003">
        <Message>无法解析</Message>
    </Item>    
    <Item id="2004">
        <Message>无法解析的宏程序指令</Message>
    </Item>    
    <Item id="2005">
        <Message>无法解析的宏运算符</Message>
    </Item>    
    <Item id="2006">
        <Message>非法代码，不支持的代码</Message>
    </Item>    
    <Item id="2007">
        <Message>单行NC代码中G指令个数超限</Message>
    </Item>    
    <Item id="2008">
        <Message>单行NC代码中M指令个数超限</Message>
    </Item>    
    <Item id="2009">
        <Message>单行NC代码中宏表达式个数超限</Message>
    </Item>    
    <Item id="2010">
        <Message>括号不匹配</Message>
    </Item>    
    <Item id="2011">
        <Message>非法的宏表达式</Message>
    </Item>    
    <Item id="2012">
        <Message>非法的宏函数操作数</Message>
    </Item>    
    <Item id="2013">
        <Message>宏表达式计算错误</Message>
    </Item>    
    <Item id="2014">
        <Message>除0错误</Message>
    </Item>    
    <Item id="2015">
        <Message>DO指令识别号超范围</Message>
    </Item>    
    <Item id="2016">
        <Message>DO-END指令不匹配</Message>
    </Item>    
    <Item id="2017">
        <Message>存在不可同行的指令</Message>
    </Item>    
    <Item id="2018">
        <Message>圆弧指令缺少参数</Message>
    </Item>    
    <Item id="2019">
        <Message>圆弧数据错误，无法自洽</Message>
    </Item>    
    <Item id="2020">
        <Message>缺少D值指定</Message>
    </Item>    
    <Item id="2021">
        <Message>缺少H值指定</Message>
    </Item>    
    <Item id="2022">
        <Message>G01、G02、G03缺少F值指定</Message>
    </Item>    
    <Item id="2023">
        <Message>子程序返回，恢复现场异常</Message>
    </Item>    
    <Item id="2024">
        <Message>主程序结束异常，子程序调用堆栈非空</Message>
    </Item>    
    <Item id="2025">
        <Message>错误的子程序接收指令，子程序必须以M99结束</Message>
    </Item>    
    <Item id="2026">
        <Message>MDA程序结束后，调用堆栈为空</Message>
    </Item>    
    <Item id="2027">
        <Message>无P字段指定子程序号</Message>
    </Item>    
    <Item id="2028">
        <Message>存在不能同行的M指令</Message>
    </Item>    
    <Item id="2029">
        <Message>预扫描执行失败</Message>
    </Item>    
    <Item id="2030">
        <Message>取消预扫描失败</Message>
    </Item>    
    <Item id="2031">
        <Message>子程序同名</Message>
    </Item>            
    <Item id="2032">
        <Message>重复的顺序号</Message>
    </Item>    
    <Item id="2033">
        <Message>找不到指定的子程序</Message>
    </Item>    
    <Item id="2034">
        <Message>跳转子程序失败</Message>
    </Item>    
    <Item id="2035">
        <Message>子程序嵌套层数过多</Message>
    </Item>    
    <Item id="2036">
        <Message>没有找到跳转点位置</Message>    
    </Item>    
    <Item id="2037">
        <Message>执行GOTO指令，跳转失败</Message>
    </Item>    
    <Item id="2038">
        <Message>END指令执行失败</Message>        
    </Item>    
    <Item id="2039">
        <Message>指令缺少参数</Message>
    </Item>    
    <Item id="2040">
        <Message>非法跳转</Message>        
    </Item>    
    <Item id="2041">
        <Message>缺少P值指定</Message>
    </Item>
  <Item id="2042">
        <Message>非法L值指定</Message>
    </Item>
  <Item id="2043">
        <Message>单行T指令个数超限</Message>
    </Item>
  <Item id="2044">
        <Message>超长词语，无法解析</Message>
    </Item>
    <Item id="2045">
        <Message>超长数值，无法解析</Message>
    </Item>
    <Item id="5000">
        <Message>取消刀具半径补偿时没有轴移动</Message>
    </Item>    
    <Item id="5001">
        <Message>刀具半径补偿非加工数据超限</Message>
    </Item>    
    <Item id="5002">
        <Message>C类刀具长度补偿轴个数超限,针对任意指定轴向进行长度补偿</Message>
    </Item>    
    <Item id="5003">
        <Message>刀具半径过大</Message>
    </Item>    
    <Item id="5004">
        <Message>刀具半径补偿过程中不可改变加工平面</Message>
    </Item>    
    <Item id="5005">
        <Message>建立或取消刀具半径补偿指令错误</Message>
    </Item>    
    <Item id="5006">
        <Message>程序结束时没有取消刀具长度补偿</Message>
    </Item>    
    <Item id="5007">
        <Message>在机床坐标系下不能使用刀具半径补偿</Message>
    </Item>    
    <Item id="5008">
        <Message>刀具长度补偿变化时指定插补指令错误</Message>
    </Item>    
    <Item id="5009">
        <Message>不支持指定曲线类型的刀具半径补偿</Message>
    </Item>    
    <Item id="5010">
        <Message>刀具半径补偿不能在建立后立即改变</Message>
    </Item>    
    <Item id="5011">
        <Message>固定循环中不能使用刀具半径补偿</Message>
    </Item>    
    <Item id="7000">
        <Message>HMI心跳丢失</Message>
    </Item>    
    <Item id="7001">
        <Message>MC模块心跳丢失</Message>
    </Item>    
    <Item id="7002">
        <Message>MI模块心跳丢失</Message>
    </Item>    
    <Item id="7003">
        <Message>HMI监控tcp连接中断</Message>
    </Item>    
    <Item id="7004">
        <Message>文件传输失败</Message>
    </Item>    
    <Item id="7005">
        <Message>MC命令包CRC校验错</Message>
    </Item>    
    <Item id="7006">
        <Message>MI命令包CRC校验错</Message>
    </Item>    

    <Item id="10000">
        <Message>负向软限位告警</Message>
    </Item>    
    <Item id="10001">
        <Message>正向软限位告警</Message>
    </Item>    
    <Item id="10002">
        <Message>位置指令过大</Message>
    </Item>    
    <Item id="10003">
        <Message>圆弧数据错误</Message>
    </Item>    
    <Item id="10004">
        <Message>指令校验错</Message>
    </Item>
    <Item id="10005">
        <Message>数据校验错</Message>
    </Item>    

    <Item id="22000">
        <Message>获取UUID失败</Message>
    </Item>    
    <Item id="22001">
        <Message>获取PMC文件失败</Message>
    </Item>    

    <Item id="30000">
        <Message>邮箱报文头错误</Message>
    </Item>    
    <Item id="30001">
        <Message>邮箱协议不支持</Message>
    </Item>    
    <Item id="30002">
        <Message>通道区域包含错误信息</Message>
    </Item>    
    <Item id="30003">
        <Message>服务不支持</Message>
    </Item>        
    <Item id="30004">
        <Message>不合法的邮箱头</Message>
    </Item>    
    <Item id="30005">
        <Message>接收的邮箱数据太短</Message>
    </Item>        
    <Item id="30006">
        <Message>从站没有空间</Message>
    </Item>    
    <Item id="30007">
        <Message>数据长度不一致</Message>
    </Item>        
    <Item id="30008">
        <Message>未知错误</Message>
    </Item>    
    <Item id="30009">
        <Message>收到了不合法的报文</Message>
    </Item>        
    <Item id="30010">
        <Message>报文数据内容太短</Message>
    </Item>    
    <Item id="30011">
        <Message>未知错误</Message>
    </Item>        
    <Item id="30012">
        <Message>无空间</Message>
    </Item>    
    <Item id="30013">
        <Message>不合法的设备建立</Message>
    </Item>        
    <Item id="30014">
        <Message>由于兼容性原因保留</Message>
    </Item>    
    <Item id="30015">
        <Message>无效状态改变请求</Message>
    </Item>        
    <Item id="30016">
        <Message>未知状态请求</Message>
    </Item>    
    <Item id="30017">
        <Message>不支持引导状态</Message>
    </Item>        
    <Item id="30018">
        <Message>固件程序无效</Message>
    </Item>    
    <Item id="30019">
        <Message>无效的邮箱配置</Message>
    </Item>        
    <Item id="30020">
        <Message>无效的邮箱配置</Message>
    </Item>    
    <Item id="30021">
        <Message>无效的SM通道配置</Message>
    </Item>        
    <Item id="30022">
        <Message>无效的输入数据</Message>
    </Item>    
    <Item id="30023">
        <Message>无效的输出数据</Message>
    </Item>        
    <Item id="30024">
        <Message>同步错误</Message>
    </Item>    
    <Item id="30025">
        <Message>SM看门狗</Message>
    </Item>        
    <Item id="30026">
        <Message>无效的SM类型</Message>
    </Item>    
    <Item id="30027">
        <Message>无效的输出配置</Message>
    </Item>    
    <Item id="30028">
        <Message>无效的输入配置</Message>
    </Item>    
    <Item id="30029">
        <Message>无效的看门狗配置</Message>
    </Item>    
    <Item id="30030">
        <Message>从站需要冷启动</Message>
    </Item>    
    <Item id="30031">
        <Message>从站需要Init状态</Message>
    </Item>        
    <Item id="30032">
        <Message>从站需要Pre-Op状态</Message>
    </Item>    
    <Item id="30033">
        <Message>从站需要safe-op状态</Message>
    </Item>        
    <Item id="30034">
        <Message>无效的输入映射</Message>
    </Item>    
    <Item id="30035">
        <Message>无效的输出映射</Message>
    </Item>        
    <Item id="30036">
        <Message>配置不一致</Message>
    </Item>    
    <Item id="30037">
        <Message>不支持Free-Run模式</Message>
    </Item>        
    <Item id="30038">
        <Message>不支持同步模式</Message>
    </Item>    
    <Item id="30039">
        <Message>Free_run需要3缓冲区模式</Message>
    </Item>        
    <Item id="30040">
        <Message>后台看门狗</Message>
    </Item>    
    <Item id="30041">
        <Message>无效的输入和输出</Message>
    </Item>        
    <Item id="30042">
        <Message>严重的同步错误</Message>
    </Item>    
    <Item id="30043">
        <Message>无效的输出FMMU配置</Message>
    </Item>        
    <Item id="30044">
        <Message>无效的输入FMMU配置</Message>
    </Item>    
    <Item id="30045">
        <Message>无效的DC 同步配置</Message>
    </Item>        
    <Item id="30046">
        <Message>无效的DC锁存配置</Message>
    </Item>    
    <Item id="30047">
        <Message>PLL错误</Message>
    </Item>        
    <Item id="30048">
        <Message>DC 同步IO错误</Message>
    </Item>    
    <Item id="30049">
        <Message>DC同步超时错误</Message>
    </Item>        
    <Item id="30050">
        <Message>无效的DC SYNC周期</Message>
    </Item>    
    <Item id="30051">
        <Message>无效的DC SYNC0周期</Message>
    </Item>        
    <Item id="30052">
        <Message>无效的DC SYNC1周期</Message>
    </Item>    
    <Item id="30053">
        <Message>AOE邮箱通信错误</Message>
    </Item>        
    <Item id="30054">
        <Message>EOE邮箱通信错误</Message>
    </Item>    
    <Item id="30055">
        <Message>COE邮箱通信错误</Message>
    </Item>    
    <Item id="30056">
        <Message>FOE邮箱通信错误</Message>
    </Item>    
    <Item id="30057">
        <Message>SOE邮箱通信错误</Message>
    </Item>    
    <Item id="30058">
        <Message>VOE邮箱通信错误</Message>
    </Item>    
    <Item id="30059">
        <Message>Eeprom 无法访问</Message>
    </Item>        
    <Item id="30060">
        <Message>Eeprom 错误</Message>
    </Item>    
    <Item id="30061">
        <Message>从站本地重启错误</Message>
    </Item>        
    <Item id="30062">
        <Message>设备标识值更新错误</Message>
    </Item>    
    <Item id="30063">
        <Message>应用控制器不存在</Message>
    </Item>        
    <Item id="30064">
        <Message>供应商特殊错误</Message>
    </Item>    
    <Item id="30065">
        <Message>分段传输时翻转位无变化</Message>
    </Item>        
    <Item id="30066">
        <Message>SDO传输超时</Message>
    </Item>    
    <Item id="30067">
        <Message>命令码无效或未知</Message>
    </Item>        
    <Item id="30068">
        <Message>内存溢出</Message>
    </Item>    
    <Item id="30069">
        <Message>不支持对某一对象的操作</Message>
    </Item>        
    <Item id="30070">
        <Message>读一个只写的对象</Message>
    </Item>    
    <Item id="30071">
        <Message>写一个只读的对象</Message>
    </Item>        
    <Item id="30072">
        <Message>子索引不支持写操作，SI0为0时写操作有效</Message>
    </Item>    
    <Item id="30073">
        <Message>SDO对可变长度对象不支持全访问</Message>
    </Item>        
    <Item id="30074">
        <Message>对象长度超过邮箱长度</Message>
    </Item>    
    <Item id="30075">
        <Message>对象映射到PDO，SDO进行块下载</Message>
    </Item>        
    <Item id="30076">
        <Message>数据对象在数据字典中不存在</Message>
    </Item>    
    <Item id="30077">
        <Message>数据对象不能映射到PDO</Message>
    </Item>        
    <Item id="30078">
        <Message>要映射对象的数量和长度超过了PDO数据长度</Message>
    </Item>    
    <Item id="30079">
        <Message>常规的参数不兼容</Message>
    </Item>        
    <Item id="30080">
        <Message>设备中常规内部不兼容</Message>
    </Item>    
    <Item id="30081">
        <Message>由于硬件错误导致操作失败</Message>
    </Item>        
    <Item id="30082">
        <Message>数据类型不匹配，服务参数长度不匹配</Message>
    </Item>    
    <Item id="30083">
        <Message>数据类型不匹配，服务参数长度过长</Message>
    </Item>    
    <Item id="30084">
        <Message>数据类型不匹配，服务参数长度过短</Message>
    </Item>    
    <Item id="30085">
        <Message>子索引不存在</Message>
    </Item>    
    <Item id="30086">
        <Message>写操作时，写入数据值超出范围</Message>
    </Item>    
    <Item id="30087">
        <Message>写入数据值太大</Message>
    </Item>        
    <Item id="30088">
        <Message>写入数据值太小</Message>
    </Item>    
    <Item id="30089">
        <Message>最大值小于最小值</Message>
    </Item>        
    <Item id="30090">
        <Message>一般错误</Message>
    </Item>    
    <Item id="30091">
        <Message>数据不可以被传输或被保存早应用程序</Message>
    </Item>        
    <Item id="30092">
        <Message>由于本地控制原因，数据不可以被传输或被保存早应用程序</Message>
    </Item>    
    <Item id="30093">
        <Message>由当前设备状态原因，数据不可以被传输或被保存早应用程序</Message>
    </Item>        
    <Item id="30094">
        <Message>对象字典动态生成错误，或没有找到对象字典</Message>
    </Item>    
    <Item id="30095">
        <Message>未知错误</Message>
    </Item>        
    <Item id="30096">
        <Message>无错误</Message>
    </Item>    
    <Item id="30097">
        <Message>一般错误</Message>
    </Item>        
    <Item id="30098">
        <Message>输入电流错误</Message>
    </Item>    
    <Item id="30099">
        <Message>内部电流错误</Message>
    </Item>        
    <Item id="30100">
        <Message>输出电流错误</Message>
    </Item>    
    <Item id="30101">
        <Message>电压错误</Message>
    </Item>        
    <Item id="30102">
        <Message>主电压错误</Message>
    </Item>    
    <Item id="30103">
        <Message>内部电压错误</Message>
    </Item>        
    <Item id="30104">
        <Message>输出电压错误</Message>
    </Item>    
    <Item id="30105">
        <Message>温度错误</Message>
    </Item>        
    <Item id="30106">
        <Message>环境温度错误</Message>
    </Item>    
    <Item id="30107">
        <Message>设备温度错误</Message>
    </Item>        
    <Item id="30108">
        <Message>设备硬件错误</Message>
    </Item>    
    <Item id="30109">
        <Message>设备软件错误</Message>
    </Item>        
    <Item id="30110">
        <Message>内部软件错误</Message>
    </Item>    
    <Item id="30111">
        <Message>用户软件错误</Message>
    </Item>    
    <Item id="30112">
        <Message>数据错误</Message>
    </Item>    
    <Item id="30113">
        <Message>附加模块错误</Message>
    </Item>        
    <Item id="30114">
        <Message>监测错误</Message>
    </Item>    
    <Item id="30115">
        <Message>通信错误</Message>
    </Item>        
    <Item id="30116">
        <Message>协议错误</Message>
    </Item>    
    <Item id="30117">
        <Message>外部错误</Message>
    </Item>        
    <Item id="30118">
        <Message>EtherCAT状态机错误</Message>
    </Item>    
    <Item id="30119">
        <Message>附加功能错误</Message>
    </Item>        
    <Item id="30120">
        <Message>制造商定义错误</Message>
    </Item>    
    <Item id="30121">
        <Message>扫描从站失败，从站个数不匹配</Message>
    </Item>        
    <Item id="30122">
        <Message>设置从站状态为INIT失败</Message>
    </Item>    
    <Item id="30123">
        <Message>设置从站初始值失败</Message>
    </Item>    
    <Item id="30124">
        <Message>初始化从站失败</Message>
    </Item>    
    <Item id="30125">
        <Message>设置从站状态为PREOP失败</Message>
    </Item>        
    <Item id="30126">
        <Message>清除多圈编码器失败</Message>
    </Item>    
    <Item id="30127">
        <Message>读取编码器值失败</Message>
    </Item>        
    <Item id="30128">
        <Message>设置对象失败</Message>
    </Item>    
    <Item id="30129">
        <Message>设置PDO失败</Message>
    </Item>        
    <Item id="30130">
        <Message>输入输出IO映射失败</Message>
    </Item>    
    <Item id="30131">
        <Message>配置DC失败</Message>
    </Item>        
    <Item id="30132">
        <Message>配置SYNC失败</Message>
    </Item>    
    <Item id="30133">
        <Message>配置主站DC时钟失败</Message>
    </Item>        
    <Item id="30134">
        <Message>配置看门狗失败</Message>
    </Item>    
    <Item id="30135">
        <Message>设置从站状态为safe-op失败</Message>
    </Item>
    <Item id="30136">
        <Message>设置从站状态为op失败</Message>
    </Item>        
    <Item id="30137">
        <Message>Safeop阶段DC时间异常</Message>
    </Item>    
    <Item id="30138">
        <Message>op阶段WKC异常</Message>
    </Item>        
    <Item id="30139">
        <Message>op阶段DC时间异常</Message>
    </Item>    
    <Item id="30140">
        <Message>op阶段从站有异常码</Message>
    </Item>
    <Item id="30150">
        <Message>从站控制模式切换有误</Message>
    </Item>
    <Item id="30151">
        <Message>不支持此设备</Message>
    </Item>
    <Item id="30152">
        <Message>轴口配置与EtherCAT从站序号不匹配</Message>
    </Item>
    <Item id="31000">
        <Message>IO板类型配置错误</Message>
    </Item>
    <Item id="31001">
        <Message>IO板未回应握手命令</Message>
    </Item>
    <Item id="31002">
        <Message>IO板回应错误握手命令</Message>
    </Item>
    <Item id="31003">
        <Message>IO板与梯图设置不匹配</Message>
    </Item>
    <Item id="31004">
        <Message>配置IO板未回应命令</Message>
    </Item>
    <Item id="31005">
        <Message>配置IO板回应错误命令</Message>
    </Item>
    <Item id="31006">
        <Message>IO板通信错误</Message>
    </Item>
    <Item id="31007">
        <Message>IO板卡出错</Message>
    </Item>
    <Item id="31100">
        <Message>PMC错误</Message>
    </Item>
    <Item id="32000">
        <Message>过流报警</Message>
    </Item>
    <Item id="33000">
        <Message>MI模块心跳超时</Message>
    </Item>
    <Item id="50000">
        <Message>监控tcp连接中断</Message>
    </Item>        
    <Item id="50001">
        <Message>未连接</Message>
    </Item>    
    <Item id="50002">
        <Message>已连接</Message>
    </Item>        

</root>
```
