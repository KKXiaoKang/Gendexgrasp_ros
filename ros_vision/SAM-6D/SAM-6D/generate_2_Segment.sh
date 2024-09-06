# Run instance segmentation model
export SEGMENTOR_MODEL=sam
#export SEGMENTOR_MODEL=fastsam

cd ./Instance_Segmentation_Model
python run_inference_custom.py --segmentor_model $SEGMENTOR_MODEL --output_dir $OUTPUT_DIR --cad_path $CAD_PATH --rgb_path $RGB_PATH --depth_path $DEPTH_PATH --cam_path $CAMERA_PATH