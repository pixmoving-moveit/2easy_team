This system only shows to yolo the image of that gets from the car and extracts the result.
From the segmented result, colors green and red are detected using HSV thresholds.
Such threasholed image is filtered to deleate noise and measured. 
From the number of pixels that remains it extracts if the traffic light is in green or red.
When red light is detected, the system informs directly.
If the green light is detected, the system waits couple of more measurements to be sure.

TODO
Sliding window to make things even easer for Yolo even when moving?
(the far away place of the traffic lights makes Yolo only detect when is too close)
