import gurobipy as gp
from gurobipy import GRB
import matplotlib.pyplot as plt
import numpy as np

# 创建模型
model = gp.Model('A_two_stage_stochastic_programming')

# 基础参数
# 使用字典直接定义参数
Ps = ['P1', 'P2', 'P3']
Pmax = {'P1': 50, 'P2': 110, 'P3': 100}  # 最大发电容量
C = {'P1': 10, 'P2': 30, 'P3': 35}  # 运行成本系数
CRU = {'P1': 16, 'P2': 13, 'P3': 10}  # 向上调节成本
CRD = {'P1': 15, 'P2': 12, 'P3': 9}  # 向下调节成本

buses = ['bus1', 'bus2']
load = {'bus1': 40, 'bus2': 100}

print("模型参数:")
print(f"发电机组: {Ps}")
print(f"各机组参数 [最大容量, 运行成本, 向上调节成本, 向下调节成本]:")
for p in Ps:
    print(f"  {p}: [{Pmax[p]}, {C[p]}, {CRU[p]}, {CRD[p]}]")
print(f"负荷: {dict(load)}")

# 创建变量
P = model.addVars(Ps, name='P')  # 日前调度发电量
RU = model.addVars(Ps, name='RU')  # 向上调节量
RD = model.addVars(Ps, name='RD')  # 向下调节量

P12 = model.addVar(lb=-100, ub=100, name='P12')  # 基础潮流
P12h = model.addVar(lb=-100, ub=100, name='P12h')  # 高负荷场景潮流
P12l = model.addVar(lb=-100, ub=100, name='P12l')  # 低负荷场景潮流

rh = model.addVars(Ps, lb=-200, name='rh')  # 高负荷场景调节量
rl = model.addVars(Ps, lb=-200, name='rl')  # 低负荷场景调节量

Lhshed = model.addVars(buses, ub=load, name='Lhshed')  # 高负荷场景负荷削减
Llshed = model.addVars(buses, ub=load, name='Llshed')  # 低负荷场景负荷削减

Ws = model.addVar(name='Ws')  # 可再生能源出力
Whspill = model.addVar(ub=50, name='Whspill')  # 高负荷场景弃风
Wlspill = model.addVar(ub=10, name='Wlspill')  # 低负荷场景弃风

# 设置目标函数
obj_da = gp.quicksum(C[p] * P[p] + CRU[p] * RU[p] + CRD[p] * RD[p] for p in Ps)  # 日前成本
obj_h = 0.6 * (gp.quicksum(C[p] * rh[p] for p in Ps) + 200 * (gp.quicksum(Lhshed[bus] for bus in buses)))  # 高场景成本
obj_l = 0.4 * (gp.quicksum(C[p] * rl[p] for p in Ps) + 200 * (gp.quicksum(Llshed[bus] for bus in buses)))  # 低场景成本
model.setObjective(obj_da + obj_h + obj_l, GRB.MINIMIZE)  # 包含所有场景成本

print("\n目标函数:")
print("min: 日前成本 + 0.6*高场景成本 + 0.4*低场景成本")
print("- 日前成本 = sum(C[p]*P[p] + CRU[p]*RU[p] + CRD[p]*RD[p])")
print("- 高场景成本 = sum(C[p]*rh[p]) + 200*sum(Lhshed[bus])")
print("- 低场景成本 = sum(C[p]*rl[p]) + 200*sum(Llshed[bus])")

# 添加约束
constr1 = model.addConstr(P['P1'] + P['P2'] - P12 + Ws == load['bus1'], name='bus1_day_ahead_balance')
constr2 = model.addConstr(P12 + P['P3'] == load['bus2'], name='bus2_day_ahead_balance')

constr3 = model.addConstr(rh['P1'] + rh['P2'] - P12h + P12 + Lhshed['bus1'] + 50 - Ws - Whspill == 0,
                          name='high_scenario_bus1_balance')
constr4 = model.addConstr(rh['P3'] + P12h - P12 + Lhshed['bus2'] == 0,
                          name='high_scenario_bus2_balance')

constr5 = model.addConstr(rl['P1'] + rl['P2'] - P12l + P12 + Llshed['bus1'] + 10 - Ws - Wlspill == 0,
                          name='low_scenario_bus1_balance')
constr6 = model.addConstr(rl['P3'] + P12l - P12 + Llshed['bus2'] == 0,
                          name='low_scenario_bus2_balance')

constrs7 = model.addConstrs((P[p] + RU[p] <= Pmax[p] for p in Ps), name='day_ahead_up_constraint')
constrs8 = model.addConstrs((P[p] - RD[p] >= 0 for p in Ps), name='day_ahead_down_constraint')

constrs9 = model.addConstrs((-RD[p] <= rh[p] for p in Ps), name='high_scenario_down_constraint')
constrs10 = model.addConstrs((rh[p] <= RU[p] for p in Ps), name='high_scenario_up_constraint')
constrs11 = model.addConstrs((-RD[p] <= rl[p] for p in Ps), name='low_scenario_down_constraint')
constrs12 = model.addConstrs((rl[p] <= RU[p] for p in Ps), name='low_scenario_up_constraint')

print("\n约束条件:")
print("1. 日前功率平衡 - bus1: P['P1'] + P['P2'] - P12 + Ws == 40")
print("2. 日前功率平衡 - bus2: P12 + P['P3'] == 100")
print("3. 高场景功率平衡 - bus1: rh['P1'] + rh['P2'] - P12h + P12 + Lhshed['bus1'] + 50 - Ws - Whspill == 0")
print("4. 高场景功率平衡 - bus2: rh['P3'] + P12h - P12 + Lhshed['bus2'] == 0")
print("5. 低场景功率平衡 - bus1: rl['P1'] + rl['P2'] - P12l + P12 + Llshed['bus1'] + 10 - Ws - Wlspill == 0")
print("6. 低场景功率平衡 - bus2: rl['P3'] + P12l - P12 + Llshed['bus2'] == 0")
print("7. 日前发电上限: P[p] + RU[p] <= Pmax[p]")
print("8. 日前发电下限: P[p] - RD[p] >= 0")
print("9-12. 场景调节约束: -RD[p] <= 调节量 <= RU[p]")

# 求解
try:
    model.write('A_two_stage_stochastic_programming.lp')
    model.optimize()

    if model.status == GRB.OPTIMAL:
        print(f"\n优化成功，目标值: {model.objVal}")
        print("\n决策变量结果:")
        results = {}
        for v in model.getVars():
            print(f"{v.varName}: {v.x:.2f}")
            results[v.varName] = v.x

        print(f"\n关键结果分析:")
        print(
            f"P1 日前发电: {results['P[P1]']:.2f}, P2 日前发电: {results['P[P2]']:.2f}, P3 日前发电: {results['P[P3]']:.2f}")
        print(f"基础潮流 P12: {results['P12']:.2f}")
        print(f"可再生能源出力 Ws: {results['Ws']:.2f}")

        # 检查功率平衡
        balance1 = results['P[P1]'] + results['P[P2]'] - results['P12'] + results['Ws']
        balance2 = results['P12'] + results['P[P3]']
        print(f"\n日前功率平衡检查:")
        print(
            f"  bus1: P1 + P2 - P12 + Ws = {results['P[P1]']:.2f} + {results['P[P2]']:.2f} - {results['P12']:.2f} + {results['Ws']:.2f} = {balance1:.2f} (应为40)")
        print(f"  bus2: P12 + P3 = {results['P12']:.2f} + {results['P[P3]']:.2f} = {balance2:.2f} (应为100)")

        # 分析P3为0的原因
        print(f"\n分析P3发电量为0的原因:")
        if results['P[P3]'] == 0:
            print("  - P3虽然容量最大(100)，但运行成本最高(35)，所以被优先停用")
            print("  - 系统优先使用成本较低的P1和P2来满足bus1的负荷")
            print("  - P3可能在某些场景下被用于调节而非基础发电")
        else:
            print("  - P3发电量非0")

        # 可视化结果
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

        # 日前发电计划
        gens = ['P1', 'P2', 'P3']
        p_vals = [results['P[P1]'], results['P[P2]'], results['P[P3]']]
        ax1.bar(gens, p_vals, color=['#1f77b4', '#ff7f0e', '#2ca02c'])
        ax1.set_title('日前发电计划')
        ax1.set_ylabel('发电量')
        for i, v in enumerate(p_vals):
            ax1.text(i, v + 1, f'{v:.2f}', ha='center')

        # 负荷削减情况
        buses_list = ['bus1', 'bus2']
        h_shed = [results['Lhshed[bus1]'], results['Lhshed[bus2]']]
        l_shed = [results['Llshed[bus1]'], results['Llshed[bus2]']]
        x = np.arange(len(buses_list))
        width = 0.35
        ax2.bar(x - width / 2, h_shed, width, label='高场景', color='#d62728')
        ax2.bar(x + width / 2, l_shed, width, label='低场景', color='#9467bd')
        ax2.set_title('负荷削减情况')
        ax2.set_ylabel('削减量')
        ax2.set_xticks(x)
        ax2.set_xticklabels(buses_list)
        ax2.legend()

        plt.tight_layout()
        plt.show()

    else:
        print(f"优化未成功，状态码: {model.status}")

except gp.GurobiError as e:
    print(f"Gurobi错误: {e}")





