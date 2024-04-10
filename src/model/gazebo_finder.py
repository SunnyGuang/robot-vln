import speech_recognition as sr
import pyttsx3
import pandas as pd
import os
import tf.transformations as tft

# Initialize the speech engine
engine = pyttsx3.init()

# COCO list of items
coco_list = [
    "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator",
    "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
]

# Initialize the recognizer
recognizer = sr.Recognizer()
def find_in_coco(text, coco_list):
    # Remove spaces and convert to lowercase for the spoken text
    normalized_text = text.replace(" ", "").lower()
    
    for item in coco_list:
        # Remove spaces and convert to lowercase for the item from the COCO list
        normalized_item = item.replace(" ", "").lower()
        
        if normalized_item in normalized_text:
            return item  # Return the original item from the list
    return None

# def find_in_coco(text, coco_list):
#     for item in coco_list:
#         if item in text.lower():
#             return item
#     return None

def find_in_csv(item, file_path):
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Cannot find the memory file at {file_path}")

    memory_df = pd.read_csv(file_path, dtype={'Class': str})
    memory_df['Class'] = memory_df['Class'].str.strip().str.lower()

    matched_object = memory_df[memory_df['Class'].str.contains(item, case=False, na=False)]
    return matched_object

def main():
    engine.say('What Can I do for you?')
    engine.runAndWait()
    #audio_data = recognizer.record(source, duration=8)

    with sr.Microphone() as source:
        print("Listening...")
        audio_data = recognizer.record(source, duration=8)  # Listen until silence is detected

    try:
        text = recognizer.recognize_google(audio_data)
        print(f"You said: {text}")

        item = find_in_coco(text, coco_list)
        if item:
            engine.say(f"You said: {text}, and you want: {item}")
            engine.runAndWait()

            matched_object = find_in_csv(item, '/home/sunny/catkin_ws/src/project/memory/gazebo_memory.csv')
            if not matched_object.empty:
                position_tuple = matched_object.iloc[0][['X', 'Y','theta']].values
                print(f"Found {item} at coordinates: X = {position_tuple[0]}, Y = {position_tuple[1]}")
        
                goal_x = round(position_tuple[0], 4)
                goal_y = round(position_tuple[1], 4)
                roll = 0
                pitch = 0
                yaw = round(position_tuple[2], 4)
                quaternion = tft.quaternion_from_euler(roll, pitch, yaw)

                pose_stamped_message = f"'{{header: {{stamp: now, frame_id: map}}, pose: {{position: {{x: {goal_x}, y: {goal_y}, z: 0.0}}, orientation: {{x: {quaternion[0]}, y: {quaternion[1]}, z: {quaternion[2]}, w: {quaternion[3]}}}}}}}'"
                os.system(f"rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped {pose_stamped_message} --once")
                
                engine.say('Found it, please follow me')
                engine.runAndWait()
                # Here, you can add the code to publish the pose to the robot

            else:
                engine.say("Sorry, I can't find it in the environment")
                engine.runAndWait()
        else:
            engine.say(f"I am not sure what you talked about, I heared {text}, Can you make sure for it?")
            engine.runAndWait()

    except sr.UnknownValueError:
        print("Could not understand the audio")
    except sr.RequestError as e:
        print(f"Could not request results; {e}")

if __name__ == "__main__":
    main()
