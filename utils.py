import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt 
import seaborn as sns
sns.set_style("whitegrid")

# Later found out this is headway
def get_leader_distances(rundf, ncars):
    leader_distances = []
    for idm in rundf.id.unique():
        leader_id = f'idm_{(int(idm.split("_")[1]) + 1) % ncars}'
        leader_dist = np.linalg.norm(
            rundf[rundf.id == idm][['x','y']].to_numpy() - rundf[rundf.id == leader_id][['x','y']].to_numpy()
            , axis=1) 
        leader_distances.append(pd.Series(leader_dist))
    return pd.concat(leader_distances, ignore_index=True)

# Later found out this is headway
def get_leader_arc_distances(rundf, ncars):
    leader_distances = []
    for idm in rundf.id.unique():
        leader_id = f'idm_{(int(idm.split("_")[1]) + 1) % ncars}'
        
        car_pos = rundf[rundf.id == idm][['x','y']].to_numpy() - 100
        leader_pos = rundf[rundf.id == leader_id][['x','y']].to_numpy() - 100
        
        diff = leader_pos - car_pos
        leader_dist = 100*np.abs(np.arctan2(diff[:,1], diff[:,0]))
        
        
        diff = rundf[rundf.id == idm][['x','y']].to_numpy() -  rundf[rundf.id == leader_id][['x','y']].to_numpy()

        leader_distances.append(pd.Series(leader_dist))
    return pd.concat(leader_distances, ignore_index=True)


def load_baseline_exp():
    expdfs = []
    for course in ['ring']:
        for ncars in range(2,24,2):
            rundf = pd.read_csv(glob.glob(f'baseline_exp_backup/exp_{course}_car_baseline_cars_{ncars}_*r2*.csv')[0]) 
            rundf['num_cars'] = ncars
            rundf['acc'] = 2
            rundf['vel'] = 30
            rundf['course'] = course
            rundf['leader_dist'] = get_leader_distances(rundf, ncars)
            rundf['leader_arc_dist'] = get_leader_arc_distances(rundf, ncars)
            expdfs.append(rundf)

        for acc in [.5,1,1.5,2,2.5,3,3.5,4]:
            rundf = pd.read_csv(glob.glob(f'baseline_exp_backup/exp_{course}_acc_baseline_acc_{acc}_*r2*.csv')[0]) 
            rundf['num_cars'] = 20
            rundf['acc'] = acc
            rundf['vel'] = 30
            rundf['course'] = course
            rundf['leader_dist'] = get_leader_distances(rundf, 20)
            rundf['leader_arc_dist'] = get_leader_arc_distances(rundf, 20)
            expdfs.append(rundf)

        for vel in range(5,35,5):
            rundf = pd.read_csv(glob.glob(f'baseline_exp_backup/exp_{course}_velocity_baseline_vel_{vel}_*r2*.csv')[0]) 
            rundf['num_cars'] = 20
            rundf['acc'] = 2
            rundf['vel'] = vel
            rundf['course'] = course
            rundf['leader_dist'] = get_leader_distances(rundf, 20)
            rundf['leader_arc_dist'] = get_leader_arc_distances(rundf, 20)
            expdfs.append(rundf)

    all_car_df = pd.concat(expdfs)
    all_car_df[all_car_df.speed < 0] = None
    all_car_df[all_car_df['leader_dist'] > 300] = None
    all_car_df = all_car_df[all_car_df['num_cars'] < 22] 
    return all_car_df

def load_constants_exp():
    rundfs = []
    for exp in glob.glob(f'hyperparam_backup/*consensus_constants*.csv'):
        ch = float(exp.split('_ch_')[1].split('_')[0])
        cv = float(exp.split('_cv_')[1].split('_')[0])
        ca = float(exp.split('_ca')[2].split('_')[0])
        rundf = pd.read_csv(exp)
        rundf['ch'] = ch
        rundf['cv'] = cv
        rundf['ca'] = ca
        rundf['num_cars'] = exp.split('cars_')[1].split('_')[0]
        rundf['emergency_brake'] = (rundf.realized_accel <  rundf.target_accel_no_noise_with_failsafe*2)
        rundfs.append(rundf)
    constdf = pd.concat(rundfs)
    return constdf