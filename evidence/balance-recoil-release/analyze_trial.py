"""Measured three-run comparison; preserves raw engagement odometry and timing."""
import json
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).parent
sys.path.insert(0,str(ROOT/'scripts'))
from analyze_balance_logs import load_rows,parse_config

files=[
    ('Sept 14 baseline','bal_20260914_233101_early-recovery-first-success.csv','#8993a0','--'),
    ('Sept 19 baseline','bal_20260919_143953_early-recovery-repeat-success.csv','#367bc3','-'),
    ('Recoil-release test','bal_20260919_151011_confirmed-recoil-release-success.csv','#00886a','-'),
]
results=[]
fig,axes=plt.subplots(3,1,figsize=(11,9),sharex=True,layout='constrained')
for label,name,color,style in files:
    path=ROOT/'telemetry_logs'/name
    all_rows=load_rows(path);cfg=parse_config(path)
    rows=[r for r in all_rows if r['state']=='2']
    v=lambda key:np.array([float(r[key]) for r in rows])
    t=(v('t_ms')-v('t_ms')[0])/1000
    flags=v('flags').astype(int);diag=v('diag_flags').astype(int)
    features=int(cfg['telemetry_features'])
    def event(mask):
        idx=np.flatnonzero(mask)
        return float(t[idx[0]]) if len(idx) else None
    events={key:event(mask) for key,mask in [
        ('arm_return',flags&32!=0),('ramp_complete',flags&64!=0),
        ('recovery',diag&0x1000!=0),('boost',diag&0x2000!=0),
        ('settled',diag&0x8000!=0)]}
    boosted=np.flatnonzero(diag&0x2000!=0)
    events['boost_last']=float(t[boosted[-1]]) if len(boosted) else None
    release=np.flatnonzero(flags&1!=0) if features&32 else np.array([],dtype=int)
    events['recoil_first']=float(t[release[0]]) if len(release) else None
    events['recoil_last']=float(t[release[-1]]) if len(release) else None
    peak=np.argmax(np.where(t<3,v('filtered_vel'),-100))
    stop=np.flatnonzero((t>t[peak])&(v('filtered_vel')<=0))[0]
    reverse=np.argmin(np.where((t>=t[stop])&(t<7),v('filtered_vel'),100))
    settled=np.flatnonzero(diag&0x8000!=0)[0]
    first4=t<4
    # Same five-second window in every run, after the transition has settled.
    quiet=(t>=t[settled]+.5)&(t<t[settled]+5.5)
    assert t[-1]>t[settled]+5.5
    feedback=np.maximum(v('feedback_age_l_ms'),v('feedback_age_r_ms'))
    total_travel_to_settle=float(np.sum(np.abs(np.diff(v('meas_drift')[:settled+1]))))
    result=dict(label=label,file=name,features=features,samples=len(all_rows),
        duration_s=(float(all_rows[-1]['t_ms'])-float(all_rows[0]['t_ms']))/1000,
        balance_s=float(t[-1]),events_after_engage_s=events,
        file_checksum=cfg['checksum'],usb_checksum=cfg['transport_fnv1a'],
        build=f"{cfg['build_date']} {cfg['build_time']}",test_note=cfg.get('test_note'),
        end_reason=cfg['end_reason'],profile_scope=cfg['profile_scope'],
        starting_trim_deg=float(cfg['stored_trim']),engage_tilt_deg=float(v('roll')[0]),
        first_forward_stop_s=float(t[stop]),first_stop_integral_deg=float(v('vel_integral')[stop]),
        reverse_peak_filtered_rad_s=float(v('filtered_vel')[reverse]),reverse_peak_s=float(t[reverse]),
        reverse_peak_measured_rad_s=float(min(v('meas_vel')[(t>=t[stop])&(t<7)])),
        stop_to_settle_s=float(t[settled]-t[stop]),
        settled_hold_travel_rad=float(v('meas_drift')[settled]),
        settled_integral_deg=float(v('vel_integral')[settled]),
        first4_peak_speed_rad_s=float(max(abs(v('meas_vel')[first4]))),
        first4_peak_command_rad_s=float(max(abs(v('motor_vel')[first4]))),
        first4_peak_travel_rad=float(max(abs(v('meas_drift')[first4]))),
        rollback_to_settle_rad=float(max(v('meas_drift')[:settled+1])-v('meas_drift')[settled]),
        accumulated_absolute_wheel_travel_to_settle_rad=total_travel_to_settle,
        final_travel_rad=float(v('meas_drift')[-1]),final_integral_deg=float(v('vel_integral')[-1]),
        matched_postsettle_speed_rms=float(np.sqrt(np.mean(v('meas_vel')[quiet]**2))),
        matched_postsettle_travel_range=float(np.ptp(v('meas_drift')[quiet])),
        matched_postsettle_samples=int(sum(quiet)),matched_postsettle_window_s=5.,
        max_angle_error_deg=float(max(abs(v('angle_err')))),
        inner_balance_max_us=int(max(v('inner_dt_max_us'))),imu_balance_max_ms=int(max(v('imu_age_ms'))),
        feedback_after50ms_max_ms=int(max(feedback[t>=.05])),
        sample_balance_max_ms=int(max(v('sample_dt_ms'))),sample_balance_p99_ms=float(np.percentile(v('sample_dt_ms'),99)),
        receiver_run_max_us=int(cfg['prof_crsf_max_us']),control_gap_max_us=int(cfg['prof_ctlgap_max_us']),
        stall_events=int(cfg['stall_events']),
        saturated_rows=int(sum((diag&1!=0)|(v('inner_sat_ticks')>0))),
        recovery_limit_rows=int(sum(diag&0x4000!=0)),imu_fault_rows=int(sum(diag&0x400!=0)),
        can_tx_fault_rows=int(sum(diag&0x800!=0)),arm_active_rows=int(sum(np.isin(v('arm_stage'),[1,2]))),
        min_bus_voltage=float(min(v('bus_voltage'))),recoil_flagged_samples=len(release))
    if len(release):
        assert np.all(flags[release]&64) and np.all(diag[release]&0x1000)
        assert np.all(v('filtered_vel')[release]*v('vel_integral')[release]<0)
        assert np.all(feedback[release]<=30)
        assert t[release[0]]>t[stop]
        # Telemetry records the updated integral. Infer gain with quantization
        # error from successive samples; ignore tiny velocity/timing intervals.
        mask=(flags&1!=0)&(abs(v('filtered_vel'))>.5)&(v('sample_dt_ms')>0)
        idx=np.flatnonzero(mask);idx=idx[idx>0]
        gains=(v('vel_integral')[idx]-v('vel_integral')[idx-1])/(.231*v('filtered_vel')[idx]*v('sample_dt_ms')[idx]/1000)
        result['inferred_flagged_gain_median']=float(np.median(gains))
        result['inferred_flagged_gain_p10_p90']=list(np.percentile(gains,[10,90]))
        result['recoil_flag_checks']='All flagged rows post-ramp, active recovery, opposite integral, fresh feedback; first after forward stop'
        for ax in axes:ax.axvspan(t[release[0]],t[release[-1]],color=color,alpha=.08)
    results.append(result)
    for ax,key in zip(axes,('filtered_vel','meas_drift','vel_integral')):
        ax.plot(t,v(key),color=color,ls=style,lw=1.8,label=label)
        ax.plot(t[settled],v(key)[settled],'o',color=color,ms=6)
        ax.grid(alpha=.2);ax.set_xlim(0,7)
    axes[0].annotate(f"{t[settled]:.2f} s settled",(t[settled],v('filtered_vel')[settled]),
                     xytext=(0,18 if label=='Sept 14 baseline' else (-30 if label=='Sept 19 baseline' else 34)),
                     textcoords='offset points',ha='center',color=color,fontsize=9)

for ax,label in zip(axes,('Filtered wheel speed (rad/s)','Wheel travel from engagement (rad)','Learned angle correction (degrees)')):
    ax.set_ylabel(label)
axes[0].axhline(0,color='black',lw=.6)
axes[0].legend(loc='upper right',fontsize=9)
axes[-1].set_xlabel('Seconds after balance engagement')
fig.suptitle('Confirmed recoil release: first physical comparison\nDots mark the unchanged 400 ms settling decision; green shading marks extra release.',fontsize=13)
fig.savefig(OUT/'trial-comparison.png',dpi=160)
(OUT/'trial-metrics.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results,indent=2))
