# DATA
dataset = 'Tusimple'
data_root = '/home/romala/catkin_ws/src/igvc_ros/igvc_perception/dl_model/dataset'

# TRAIN
epoch = 100
batch_size = 32
optimizer = 'SGD'    #['SGD','Adam']
# learning_rate = 0.1
learning_rate = 0.05
weight_decay = 0.0001
momentum = 0.9

scheduler = 'cos'     #['multi', 'cos']
# steps = [50,75]
gamma  = 0.1
warmup = 'linear'
warmup_iters = 100

# NETWORK
backbone = '18'
griding_num = 100
use_aux = False

# LOSS
sim_loss_w = 1.0
shp_loss_w = 0.0

# EXP
note = ''

log_path = '/home/sree/ros_ws/src/igvc_ros/logs'

# FINETUNE or RESUME MODEL PATH
finetune = None
resume = None

# TEST
test_model = "/home/sree/ros_ws/src/igvc_ros/igvc_perception/dl_model/configs/tusimple_18.pth"
test_work_dir = '/media/warlord/workspace/AONS/src/pc_visualizer/src/dl_model/work_dir'


num_lanes = 4
