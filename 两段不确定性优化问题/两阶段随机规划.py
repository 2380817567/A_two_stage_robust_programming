import gurobipy as gp
from gurobipy import GRB

model = gp.Model('A_two-stage_stochastic_programming')

#基础参数
Ps, Pmax, C, CRU, CRD = gp.multidict ({
    'P1':[50,10,16,15],
    'P2':[110,30,13,12],
    'P3':[100,35,10,9]
})

buses,load = gp.multidict ({
    'bus1':40,
    'bus2':100
})

#创建变量
P = model.addVars(Ps,name='P')
RU = model.addVars(Ps,name='RU')
RD = model.addVars(Ps,name='RD')

P12 = model.addVar(lb=-100,ub=100,name='P12')
P12h = model.addVar(lb=-100,ub=100,name='P12h')
P12l = model.addVar(lb=-100,ub=100,name='P12l')

rh = model.addVars(Ps,lb=-200,name='rh')
rl = model.addVars(Ps,lb=-200,name='rl')

Lhshed = model.addVars(buses,ub=load,name='Lhshed')
Llshed = model.addVars(buses,ub=load,name='Llshed')

Ws = model.addVar(name='Ws')
Whspill = model.addVar(ub=50,name='Whspill')
Wlspill = model.addVar(ub=10,name='Wlspill')

#设置目标
obj_da = gp.quicksum(C[p]*P[p]+CRU[p]*RU[p]+CRD[p]*RD[p] for p in Ps)
obj_h = 0.6*(gp.quicksum(C[p]*rh[p] for p in Ps)+200*(gp.quicksum(Lhshed[bus] for bus in buses)))
obj_l = 0.4*(gp.quicksum(C[p]*rl[p] for p in Ps)+200*(gp.quicksum(Llshed[bus] for bus in buses)))
model.setObjective(obj_da+obj_h+obj_l, GRB.MINIMIZE)

#添加约束
model.addConstr(P['P1']+P['P2']-P12+Ws==load['bus1'],name='bus1_day_ahead_balance')
model.addConstr(P12+P['P3']==load['bus2'],name='bus2_day_ahead_balance')

model.addConstr(rh['P1']+rh['P2']-P12h+P12+Lhshed['bus1']+50-Ws-Whspill==0
                ,name='high_scenario_bus1_balance')
model.addConstr(rh['P3']+P12h-P12+Lhshed['bus2']==0
                ,name='high_scenario_bus2_balance')

model.addConstr(rl['P1']+rl['P2']-P12l+P12+Llshed['bus1']+10-Ws-Wlspill==0
                ,name='low_scenario_bus1_balance')
model.addConstr(rl['P3']+P12l-P12+Llshed['bus2']==0
                ,name='low_scenario_bus2_balance')

model.addConstrs((P[p]+RU[p]<=Pmax[p] for p in Ps),name='day_ahead_up_constraint')
model.addConstrs((P[p]-RD[p]>=0 for p in Ps),name='day_ahead_down_constraint')

model.addConstrs((-RD[p]<=rh[p] for p in Ps),name='high_scenario_down_constraint')
model.addConstrs((rh[p]<=RU[p] for p in Ps),name='high_scenario_up_constraint')
model.addConstrs((-RD[p]<=rl[p] for p in Ps),name='low_scenario_down_constraint')
model.addConstrs((rl[p]<=RU[p] for p in Ps),name='low_scenario_up_constraint')

#求解
model.write('A_two-stage_robust_programming.lp')
model.optimize()
# P1=50,P2=40,P3=40,R3D=40,r3h=-40,Ws=10,obj=2620
for v in model.getVars():
    print(v.varName,v.x)






































