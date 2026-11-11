"""Quantitative figures from preserved final benchmark data; no simulation."""
from pathlib import Path
import csv,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
ROOT=Path(__file__).resolve().parent
DATA=ROOT/'reports/walking_async_final'
OUT=ROOT/'figures'
OUT.mkdir(exist_ok=True)
font_manager.fontManager.addfont('/mnt/c/Windows/Fonts/arialbd.ttf')
plt.rcParams.update({'font.family':'Arial','font.weight':'bold','font.size':13,
 'axes.labelweight':'bold','axes.titleweight':'bold','axes.labelsize':14,'axes.titlesize':14,
 'xtick.labelsize':12,'ytick.labelsize':12,'legend.fontsize':12,'legend.frameon':False,
 'axes.grid':False,'axes.spines.top':True,'axes.spines.right':True,'axes.linewidth':1.4,
 'xtick.direction':'in','ytick.direction':'in','xtick.top':True,'ytick.right':True,
 'xtick.major.width':1.4,'ytick.major.width':1.4,'lines.linewidth':3.2,'lines.linestyle':'-',
 'figure.facecolor':'white','axes.facecolor':'white','savefig.facecolor':'white',
 'svg.fonttype':'none','pdf.fonttype':42})
BLUE='#5B83BC';CORAL='#FC9262';PINK='#E9A4C1';LIGHT='#94BFDC'
def save(fig,name):
    fig.savefig(OUT/f'{name}.png',dpi=300)
    fig.savefig(OUT/f'{name}.svg')
    fig.savefig(OUT/f'{name}.pdf')
    fig.savefig(OUT/f'{name}.tiff',dpi=600,pil_kwargs={'compression':'tiff_lzw'})
    plt.close(fig)
def label(ax,s):
    ax.text(-.09,1.05,s,transform=ax.transAxes,fontsize=17,fontweight='bold')
    ax.grid(False)
def trace(name):
    with (DATA/name/'trace.csv').open() as f:rr=list(csv.DictReader(f))
    return rr,{k:np.array([float(r[k]) for r in rr]) for k in rr[0] if k!='mode'}
bundle=json.loads((DATA/'benchmark.json').read_text())
metrics={r['case']['name']:r for r in bundle['results']}
fig,axs=plt.subplots(1,2,figsize=(12,4.9))
fig.subplots_adjust(left=.085,right=.98,bottom=.17,top=.79,wspace=.3)
x=np.arange(2)
for j,(key,ylabel) in enumerate([('height_rmse_mm','Height RMSE (mm)'),('velocity_rmse_mps','Velocity RMSE (m/s)')]):
    for i,(variant,color,display) in enumerate([('baseline',BLUE,'Fixed nominal model'),('compensated',CORAL,'Estimated payload compensation')]):
        y=[metrics[f'load{v}_{variant}'][key] for v in [10,20]]
        axs[j].bar(x+(i-.5)*.34,y,width=.32,color=color,label=display)
    axs[j].set_xticks(x,['10% payload','20% payload']);axs[j].set_ylabel(ylabel);axs[j].set_ylim(bottom=0)
    label(axs[j],chr(97+j))
fig.legend(*axs[0].get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.52,.98),ncol=2)
save(fig,'01_walking_results')
fig,axs=plt.subplots(3,1,figsize=(12.5,9.1),sharex=True)
fig.subplots_adjust(left=.10,right=.98,bottom=.08,top=.87,hspace=.25)
for name,color,display in [('load20_baseline',BLUE,'Fixed nominal model'),('load20_compensated',CORAL,'Estimated payload compensation')]:
    rr,a=trace(name);t=a['time_s']
    axs[0].plot(t,a['vx_mps'],color=color,label=display)
    axs[1].plot(t,a['height_error_m']*1000,color=color)
    axs[2].plot(t,a['payload_estimate_kg'],color=color)
axs[0].step(t,np.where(t>4.,.1,0.),where='post',color='#20242A',lw=2.2,label='Reference')
axs[1].plot(t,np.zeros_like(t),color='#20242A',lw=2.2)
axs[2].plot(t,np.full_like(t,metrics['load20_compensated']['case']['load']*15.206408),color='#20242A',lw=2.2)
for i,(ax,y) in enumerate(zip(axs,['Forward speed (m/s)','Height error (mm)','Estimated payload (kg)'])):
    ax.set_ylabel(y);label(ax,chr(97+i));ax.set_xlim(0,12)
axs[-1].set_xlabel('Time (s)')
fig.legend(*axs[0].get_legend_handles_labels(),loc='upper center',bbox_to_anchor=(.52,.99),ncol=2)
save(fig,'02_loaded_walking')
fig,axs=plt.subplots(3,2,figsize=(13,9.2))
fig.subplots_adjust(left=.10,right=.98,bottom=.08,top=.94,hspace=.5,wspace=.35)
rr,a=trace('load20_compensated')
for i,(leg,color) in enumerate(zip(['FL','FR','RL','RR'],[BLUE,CORAL,PINK,LIGHT])):
    ax=axs.flat[i];ax.plot(a['time_s'],(a[leg+'_z_m']-.022)*1000,color=color)
    ax.set_title(leg+' foot',pad=12);ax.set_xlim(0,12);ax.set_ylabel('Clearance (mm)')
    label(ax,chr(97+i))
rr,a=trace('solver_failure_stop')
axs[2,0].plot(a['time_s'],a['vx_mps'],color=BLUE)
axs[2,0].set_ylabel('Forward speed (m/s)');axs[2,0].set_title('Injected solver failure',pad=12)
states=np.array([{'RUN':0,'STOPPING':1,'STOPPED':2}[r['mode']] for r in rr])
axs[2,1].step(a['time_s'],states,where='post',color=CORAL)
axs[2,1].set_yticks([0,1,2],['RUN','STOPPING','STOPPED'])
axs[2,1].set_ylim(-.25,2.25);axs[2,1].set_title('Supervisor state',pad=12)
for i,ax in enumerate(axs[2]):ax.set_xlabel('Time (s)');label(ax,chr(101+i))
save(fig,'03_contacts_and_stop')
contract={'backend':'Python/matplotlib','archetype':'quantitative grids',
'claim':'In the fixed forward-walking protocol, estimated payload compensation reduces load-induced height error while preserving bounded torque and physical foot transitions; injected planner failure leads to a stopped stance.',
'protocol':'4s pre-stand +8s 0.1m/s forward; metric window5-12s; one paired initial state per case',
'panel_data':{'01':'10%/20% paired baseline and compensated metrics; no nominal baseline walking run is invented',
'02':'all 3000 rows from each 20% load trajectory; zero height/command/true payload references',
'03':'all 3000 rows from compensated20% foot trajectories and failure-stop trace'},
'other_cases':'nominal and noisy-push case are retained in benchmark.json and README table; not duplicated in selected trace panels',
'uncertainty':'none; no confidence intervals or significance claims from single trajectories',
'style':'Arial Bold, solid curves, closed axes, inward ticks, white background without grid or phase guide lines; no footer paragraphs',
'exports':'PNG300dpi, editable SVG, TrueType PDF, TIFF600dpi; engineering presentation size, not journal column width'}
(OUT/'figure_contract.json').write_text(json.dumps(contract,indent=2)+'\n')
print('Saved three figure bundles from existing measurements')
