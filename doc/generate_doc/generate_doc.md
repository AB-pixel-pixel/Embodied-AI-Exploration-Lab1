pandoc "~/catkin_ws/2. ros_experiment.en.md" -o "~/catkin_ws/2. ros_experiment.en.pdf" --pdf-engine=xelatex --variable mainfont="Segoe UI" --variable CJKmainfont="Microsoft YaHei" --variable fontsize=12pt --variable geometry:"margin=2.5cm" --variable linestretch=1.4 -H header.tex --resource-path="~/catkin_ws"



例子：

```
pandoc "~/catkin_ws/2. ros_experiment.en.md" -o "~/catkin_ws/2. ros_experiment.en.pdf" --pdf-engine=xelatex --variable mainfont="Segoe UI" --variable CJKmainfont="Microsoft YaHei" --resource-path="~/catkin_ws"
```


```bash
pandoc "/home/ling/catkin_ws/2_ros_experiment.md" -o "~/catkin_ws/2. ros_experiment.en.pdf" --pdf-engine=xelatex --variable mainfont="Segoe UI" --variable CJKmainfont="Microsoft YaHei" --variable fontsize=12pt --variable geometry:"margin=2.5cm" --variable linestretch=1.4 -H "/home/ling/catkin_ws/generate_doc/header.tex" --resource-path="~/catkin_ws"
```

``` bash
pandoc "/home/ling/catkin_ws/2_ros_experiment.md"   -o ~/catkin_ws/"2. ros_experiment.en.pdf"   --pdf-engine=xelatex   --variable mainfont="DejaVu Sans"   --variable CJKmainfont="Noto Sans CJK SC"   --variable fontsize=12pt   --variable geometry:"margin=2.5cm"   --variable linestretch=1.4   --resource-path="/home/ling/catkin_ws"
```


