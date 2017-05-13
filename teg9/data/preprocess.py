replace_dic = {"REPO":'kzpy3',"TEG":'teg9'}; rd = replace_dic; sr = str_replace

exec(sr("""
from REPO.vis import *
from REPO.TEG.data.preprocess_bag_data import *
from REPO.TEG.data.preprocess_Bag_Folders import *
from REPO.teg7.data.Bag_File import *
import shutil
	"""),rd)

bag_folders_src_location = sys.argv[1]
bag_folders_dst = sys.argv[2]
NUM_STATE_ONE_STEPS = sys.argv[3]

assert(is_number(NUM_STATE_ONE_STEPS))

bag_folders_src = opj(bag_folders_src_location,'new' )
bag_folders_dst_rgb1to4_path = opj(bag_folders_dst,'rgb_1to4')
bag_folders_dst_meta_path = opj(bag_folders_dst,'meta')

runs = sgg(opj(bag_folders_src,'*'))
assert(len(runs) > 0)

tb = '\t'

cprint('Preliminary check of '+bag_folders_src)
cprint("	checking bag file sizes and run durations")

for r in runs:
	bags = sgg(opj(r,'*.bag'))
	cprint(d2s(tb,fname(r),len(bags)))
	mtimes = []
	for b in bags:
		bag_size = os.path.getsize(b)
		mtimes.append(os.path.getmtime(b))
		if bag_size < 0.99 * 1074813904:
			cprint(d2s('Bagfile',b,'has size',bag_size,'which is below full size.'),'red')
			unix('mv '+b+' '+b+'.too_small')
	mtimes = sorted(mtimes)
	run_duration = mtimes[-1]-mtimes[0]
	print run_duration
	assert(run_duration/60./60. < 3.) # If clock set incorrectly, this can change during run leading to year-long intervals
	cprint(d2s(r,'is okay'))

for r in runs:
	preprocess_bag_data(r)

bag_folders_transfer_meta(bag_folders_src,bag_folders_dst_meta_path)

bag_folders_save_images(bag_folders_src,bag_folders_dst_rgb1to4_path)

if True:
	preprocess_Bag_Folders(bag_folders_dst_meta_path,bag_folders_dst_rgb1to4_path,NUM_STATE_ONE_STEPS=NUM_STATE_ONE_STEPS,graphics=False,accepted_states=[1,3,5,6,7])#,pkl_name='Bag_Folder_90_state_one_steps.pkl')
if False:
	preprocess_Bag_Folders(bag_folders_dst_meta_path,bag_folders_dst_rgb1to4_path,NUM_STATE_ONE_STEPS=NUM_STATE_ONE_STEPS,graphics=False,accepted_states=[1,3,5,6,7])
if False:
	preprocess_Bag_Folders(bag_folders_dst_meta_path,bag_folders_dst_rgb1to4_path,NUM_STATE_ONE_STEPS=NUM_STATE_ONE_STEPS,graphics=False,accepted_states=[1,3,5,6,7],pkl_name='Bag_Folder_60_state_one_steps.pkl')

os.rename(bag_folders_src,opj(bag_folders_src_location,'processed'))

