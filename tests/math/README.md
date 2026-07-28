# Server-only mathematical verification tests

本目录预留给障碍证书数学实现的服务器验证，不在本机运行。

计划覆盖：

- 研究契约中的离散系统、controller、domain、initial/goal/unsafe sets 与噪声；
- polynomial basis、系数顺序、`W(x)`、一步期望和 decrease residual；
- 3–5 个手算 golden cases，以及 offline 与 runtime evaluator 一致性；
- applicable、violated、not-applicable、out-of-domain 和 numerical-error 分流；
- system/controller/scenario/noise/state-schema hash 不匹配时拒绝加载；
- solver success、infeasible、unknown 和数值退化证据；
- 独立 validator 不复用未经检查的生成结果；
- 证明义务、求解器版本、容差、随机种子和原始日志一并留档。

当前仅建立测试边界，没有已运行或已通过的数学验证。数值测试本身不等于形式化证明；只有明确假设、证明义务和可复核证据链完整时，才能提出相应的形式化结论。
