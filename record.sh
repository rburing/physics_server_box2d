sed -i '' 's/physics_engine=.*/physics_engine="GodotPhysics2D"/' "demo/project.godot" 
/Applications/Godot-2.app/Contents/MacOS/Godot --path ./demo --debug-collisions --write-movie recording/GodotPhysics2D.avi
sed -i '' 's/physics_engine=.*/physics_engine="Box2D"/' "demo/project.godot" 
/Applications/Godot-2.app/Contents/MacOS/Godot --path ./demo --debug-collisions --write-movie recording/Box2D.avi
ffmpeg -i demo/recording/Box2D.avi -i demo/recording/GodotPhysics2D.avi -filter_complex \
"[0:v]setpts=PTS-STARTPTS, pad=iw*2:ih[bg]; \
[1:v]setpts=PTS-STARTPTS[fg]; [bg][fg]overlay=w" output.gif