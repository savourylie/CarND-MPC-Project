# MPC Project

## Introduction
In this project we use MPC (Model Predictive Control) controller to make the car drive itself in the simulator. There's simulated 100 millisecond latency between the actuation and the vehicle response.

## How MPC controller works in plain English
The idea behind MPC is simple. The controller is given a **path** to follow, which is planned out by the path planner (which would be another topic). The path is represented by **waypoints**. So the first job of the MPC is to **connect the dots using a polynomial** so that it knows where the car should be for any given time. Once the target is clear, MPC then has to determine a **window of prediction** where MPC predicts where the car is going to be sometime in the future. We can think of this path within this window as another function, and the **distance** between this function and the polynomial we talked about earlier as the **cost**, which we want to minimize. Given the current state and actuation, we know that we can use Newton's laws of motion to calculate where the car's going to be for any given time in the future, which sounds a bit too good to be true. **All of this is legit, only if all of our information is legit**. However, in the real world, data is noisy and also limited, because anything can happen on the road and we can't account for all that. We know that actuation (steering and thrust) is something we can control, so they are the parameters of our function of predicted path. Since we have determined our cost function as the distance between the said two functions, we can solve this non-linear optimization problem to find the best actuation within this window. We can choose this window to be very small so that this fortune-telling thing isn't too far off. However, no matter how small this window is, we know two things:
1. The further into the future, the less accurate we can predict.
2. We are really only interested in what we need to do NOW.

So although we can optimize for actuation for any given time within this window, the most accurate and relevant information is the actuation of now. We can discard the rest and get them next time when we need to determine our actuation using the same method.

Picking the size of this window is tricky. You don't want it to be too large, and you don't want it to be too small. If it is too large, your prediction is basically nonsense and all this optimization is just optimizing nonsense. But if you make it too small, then you can't account for some sharp turns if you're going really fast, because you're too short-sighted. Another thing to think about is how often you want to change your actuation. If you do it too often, you're wasting your computing power because there's latency in the system so basically within that delta t the actuation is all the same. But if you only make the change once in a while, you'll be in the water.

## How MPC works in pseudo code with annotation
Here is the MPC algorithm from Udacity SDCND:

Setup:

1. **Define the length of the trajectory, `N`, and duration of each timestep, `dt`**. This length of trajectory is just the window we talked about earlier, and `dt` is the reciprocal of "how often we change actuation". The length of the trajectory, let's called it `W` is the multiplication of `N` and `dt`.
2. **Define vehicle dynamics and actuator limitations along with other constraints.** This is the governing equation we talked about earlier. The motion of the car is governed by Newton's law of motion, so if your current speed and location and orientation are determined, and you know your thrust, and steering, then the status of the car in the next timestamp is determined. We call this the constraints. There are also some other constraints we need to account for like the range of the thrust and steering.
3. **Define the cost function.** We talked about this earlier. The cost is bad, so we want it to be small. Here we know the target is to follow that path laid out in front of us, so any deviation is bad.

Loop:

1. **We pass the current state as the initial state to the model predictive controller.** This is obvious. No comment.
2. **We call the optimization solver. Given the initial state, the solver will return the vector of control inputs that minimizes the cost function. The solver we'll use is called *Ipopt*.** Ipopt is a non-linear solver that can solve problems of optimization with constraints. Good stuff.
3. **We apply the first control input to the vehicle.** This is just what I said earlier. The rest of the controls are kind of like fortune-telling. We took the time to calculate all of them but don't feel bad discarding them, we'll get something better in the next timestamp.
4. **Back to 1.** That's why we call it a loop.

## Timestep length and frequency (`N` and `dt`)
I think it's obvious that 0.1 is a good choice for `dt` since that's the latency, so anything smaller than that doesn't make sense. On the other hand, in the real, continuous world, `dt` is essentially 0, and we want our computer to be as close to this as possible without comprising the performance, so I think 0.1 is good.

Once `dt` is determined, we just need to adjust `N` so that `N * dt` is reasonable. I started with `N = 10`, but the car drives in a way that's very close to what I would called dangerous, so I tried to twiddle it manually to see if there's anything better in the vicinity. ~~When `N = 8`, the car is driving more radically as it would fail to see harsh turns. At `N = 15`, the car would think the world is too dangerous and won't go anywhere in the sharp turns. In the end, I settled for `N = 12`~~ . My original solution was fine on my computer but the reviewer came back saying it wasn't working. I tried to run it on one of my slower computer and realized that it wouldn't work when there was a heavy lag. I made some modification and changed `N` back to `10`.


## Latency
One way I can think of to account for latency is you try to predict when the car would be when the response comes. We know the latency is 100 milliseconds so that means the response would come in 100 milliseconds each time you change your actuation. Since we know that that's when the response would come, we know that the status of the car at that time would be different then that of now, so we can account for that using Newton's law of motion again. So here's how I calculated the delay status:

```cpp
double delta_t = 0.1;
double current_px = v * delta_t;
double current_py = 0;
double current_psi = (v * (-delta) / Lf) * delta_t;
double current_v = v + a * delta_t;
double current_cte = cte + v * sin(epsi) * delta_t;
double current_epsi = epsi + (v * (-delta) / Lf) * delta_t;
```

Theoretically this should work, but well, for some reason it doesn't. While passing the current status works pretty well, with this delayed implementation the car would go haywire at harsh turns. ~~I think there might be some bugs in my optimizer or FG but I couldn't find it. So in the end I had go back using current status, and slow down the car so the this delay is negligible.~~ I realised the bug was in my implementation of accounting for the timestamp. I used `clock()` time divided by `CLOCKS_PER_SEC`. However this doesn't really give you the real clock time in C++, so when I realized this I changed this and used `high_resolution_clock::now()` which gives far better accuracy. I then rolled back to the revised motion model and it seemed to work well, and worked on slower computers as well.