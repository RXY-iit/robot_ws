当前test csv 结果的问题：
1.ask_human 应该把选项也列出 用于检查是否合理
2. green_door, blue_cart 这种复合结果不应该是单一对象，还是说这个不在parser matrix里检测 而是交给peception matrix？
3.Parser Matrix需要追加更多的testcase：
  3.1 Deterministic 的所有可能性
  3.2 每一个 Deterministic 需要追加 3 个Embedding Ambiguous的testcase
  3.3 每一个 Deterministic 需要追加 3 个LLM的testcase
4. Perception Matrix需要追加更多的testcase：
  4.1 ocr, yolo, color, uncertain 分别需要追加更多的testcase，在testcase设计时不应该考虑可以识别什么，用什么格式，应该更独立地忘记当前地图和场景设计更多的testcase。因为percception是一个通用的skill，未来应该有移植能力，所以设计应该尽可能地全面。
  4.2 组合任务设计更多testcase，同上， 应该和现有场景无关，已经定义的内容无关
  4.3 Perception Matrix的test设计可以划分场景，但是不同场景应该跑一样的test input，因为用的perception skill是相同的。能不能识别出box和场景无关，仅和yolo有关。yolo，color，ocr应该都定义了各自的能力范围，比如：
  car：current yolo capable
  notebook: current yolo not capable, but object like.
  love/ dangerous: not object like.
  red: hsv capable
  golden: current hsv not capable
  water blue: current hsv not capable

  检查对象的object提取是前一步的Parser Matrix根据commond分离出来的。
  比如：
  check for red box -> arser Matrix -> "red box" object
  check for red rectangle-> arser Matrix -> "red rectangle" object
  check for anything dangrous?-> arser Matrix -> "anything dangrous?" object
  check for salfjogasf-> arser Matrix -> "salfjogasf" object
  then at perception step:
  red box > red, box (konw color, konw object)> next step (trigger capture step)
  red rectangle > red, rectangle (konw color, unkonw object)> check red + ask human (show "rectangle unkonw") (trigger capture step)
  anything dangrous > unkonw object > feedback to huamn and finish (no need trigger capture step)
  salfjogasf > unkonw object feedback to huamn and finish (no need trigger capture step)
  思考：object like or not maybe not important. 因为如果不能检测都是输出unknow object就好了，对吗？
  原则就是检测能检测的部分，能检测的对象是确定的，没有能检测的对象时（任务本身超出了能力范围），feedback to human，重新指定任务而不用trigger 下一步操作。


  1. agree
  2. agree
  3. by task_type is ok
  4. agree. also yolo, ocr, hsv, 分别可以检测什么应该是由模型决定的，这个是确定的。未来模型有扩展的话，对应的总的Capability Matrix也应该可以扩展。 具体可以检测什么需要去查对应模型的资料补全。

addition:
i want actually refactor DeterministicParser/HybridParser/PerceptionSkills , 先整体重写逻辑里的漏洞，重新设计testcases。
 "partial" (color known, object unknown) always mean "attempt + ask_human," 不仅是颜色partially，其他情况也是，比如 konw object, unknown color, know ocr object, unknow color.


---
现在的Embedding Ambiguous都会指向 ask_human. 
我理解的运行逻辑：
Deterministic： stop
Embedding ： 少し待ってください/ stop TASK please
LLM Fallback ：今日はここまでにして -> stop