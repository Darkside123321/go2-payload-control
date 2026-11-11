"""检查学员核心计算；从项目内算法文件加载，不访问仿真实际载荷。"""
from pathlib import Path
import importlib.util,math,sys
CORE=Path(__file__).resolve().parent/'learner_core.py'
spec=importlib.util.spec_from_file_location('learner_core_student',CORE)
core=importlib.util.module_from_spec(spec);spec.loader.exec_module(core)
m=15.206408;g=9.81
cases=[
 ('无额外载荷：估计应为0 kg',core.payload_from_support,(m*g,m,g),0.),
 ('静态附加2 kg：估计应为2 kg',core.payload_from_support,((m+2)*g,m,g),2.),
 ('小负偏差保留到估计器外层处理',core.payload_from_support,((m-.1)*g,m,g),-.1),
 ('滤波：0.2 kg旧值、2.2 kg新值、alpha=0.1',core.lowpass,(.2,2.2,.1),.4),
 ('alpha=0时保持旧值',core.lowpass,(.2,2.2,0.),.2),
 ('alpha=1时采用新值',core.lowpass,(.2,2.2,1.),2.2),
 ('1 kg载荷的总补偿力应为9.81 N',core.weight_compensation,(1.,g),9.81),
 ('0载荷补偿为0 N',core.weight_compensation,(0.,g),0.)]
pending=0;failed=0
for name,fun,args,expected in cases:
    try:
        actual=fun(*args)
        ok=actual is not None and math.isfinite(float(actual)) and math.isclose(actual,expected,rel_tol=1e-8,abs_tol=1e-8)
        print(('PASS ' if ok else 'FAIL ')+name+(f'；实际={actual}' if not ok else ''))
        failed+=int(not ok)
    except NotImplementedError:
        pending+=1;print('待填写 '+name)
    except Exception as e:
        failed+=1;print('ERROR '+name+'：'+str(e))
print(f'通过={len(cases)-failed-pending}，待填写={pending}，失败={failed}')
sys.exit(2 if pending else 1 if failed else 0)
