
import os

def main():
    waves_path = "gz_ws/src/asv_wave_sim/gz-waves-models/worlds/waves.sdf"
    course_path = "course.xml"
    
    with open(waves_path, "r") as f:
        waves_content = f.read()
        
    with open(course_path, "r") as f:
        course_content = f.read()
        
    placeholder = "<!-- ... (Due to tool limits, I cannot paste 1800 lines here. I will use a different approach) ... -->"
    
    if placeholder in waves_content:
        new_content = waves_content.replace(placeholder, course_content)
        with open(waves_path, "w") as f:
            f.write(new_content)
        print("Successfully replaced placeholder with course content.")
    else:
        print("Placeholder not found!")

if __name__ == "__main__":
    main()
