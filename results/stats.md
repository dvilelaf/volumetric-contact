# Battery statistics

Source: `battery.csv`. 11640 runs, 33 scenarios, 24 methods. Error floor 0.001 (a practical tie); nan, timeout and failed count as infinite error; the uncapped paired comparisons leave those pairs out (column `excl`), the robust ones count them at the cap.

Runs by status: failed 11, nan 145, ok 11467, timeout 17.

## Pass rates (base variant)

Share of scenarios with error at or below 1 / 5 / 20 %. Class C: laws and the Winkler native only.

### Class A

| method | n | ≤1 % | ≤5 % | ≤20 % | median | geo-mean | failed |
|---|---|---|---|---|---|---|---|
| chrono-gonthier | 21 | 48% | 67% | 90% | 0.0277 | 0.0232 | 0 |
| chrono-gonthier-thesis | 21 | 24% | 52% | 76% | 0.039 | 0.12 | 0 |
| chrono-distributed | 21 | 57% | 76% | 90% | 0.00596 | 0.0186 | 0 |
| chrono-distributed-vertex | 21 | 43% | 62% | 90% | 0.0346 | 0.0328 | 0 |
| chrono-distributed-centroid | 21 | 43% | 62% | 86% | 0.0346 | 0.037 | 0 |
| chrono-pcm | 21 | 19% | 48% | 67% | 0.135 | 0.149 | 0 |
| chrono-native | 21 | 38% | 48% | 76% | 0.0569 | 0.0328 | 0 |
| chrono-native-nsc | 21 | 48% | 52% | 62% | 0.0143 | 0.0216 | 1 |
| drake-gonthier | 21 | 48% | 67% | 90% | 0.0277 | 0.0233 | 0 |
| drake-gonthier-thesis | 21 | 24% | 52% | 76% | 0.0348 | 0.0683 | 1 |
| drake-distributed | 21 | 57% | 76% | 90% | 0.00595 | 0.0186 | 0 |
| drake-distributed-vertex | 21 | 43% | 62% | 90% | 0.0346 | 0.0328 | 0 |
| drake-distributed-centroid | 21 | 43% | 62% | 86% | 0.0346 | 0.0369 | 0 |
| drake-pcm | 21 | 24% | 48% | 67% | 0.135 | 0.147 | 0 |
| drake-native | 21 | 33% | 62% | 71% | 0.0199 | 0.032 | 1 |
| drake-native-point | 21 | 57% | 62% | 81% | 0.00536 | 0.025 | 0 |
| mujoco-gonthier | 21 | 48% | 67% | 90% | 0.0277 | 0.0232 | 0 |
| mujoco-gonthier-thesis | 21 | 24% | 52% | 76% | 0.0348 | 0.0694 | 1 |
| mujoco-distributed | 21 | 57% | 76% | 90% | 0.00596 | 0.0186 | 0 |
| mujoco-distributed-vertex | 21 | 43% | 62% | 90% | 0.0346 | 0.0328 | 0 |
| mujoco-distributed-centroid | 21 | 43% | 62% | 86% | 0.0346 | 0.037 | 0 |
| mujoco-pcm | 21 | 24% | 48% | 67% | 0.135 | 0.148 | 0 |
| mujoco-native | 21 | 52% | 62% | 81% | 0.00864 | 0.0195 | 0 |
| mujoco-native-default | 21 | 19% | 33% | 52% | 0.19 | 0.0669 | 0 |

### Class B

| method | n | ≤1 % | ≤5 % | ≤20 % | median | geo-mean | failed |
|---|---|---|---|---|---|---|---|
| chrono-gonthier | 5 | 20% | 60% | 100% | 0.0387 | 0.0311 | 0 |
| chrono-gonthier-thesis | 5 | 0% | 40% | 80% | 0.0905 | 0.072 | 0 |
| chrono-distributed | 5 | 80% | 100% | 100% | 0.000746 | 0.00182 | 0 |
| chrono-distributed-vertex | 5 | 40% | 60% | 80% | 0.016 | 0.0192 | 0 |
| chrono-distributed-centroid | 5 | 40% | 80% | 100% | 0.016 | 0.00954 | 0 |
| chrono-pcm | 5 | 40% | 60% | 100% | 0.0217 | 0.0125 | 0 |
| chrono-native | 5 | 0% | 20% | 60% | 0.0993 | 0.0696 | 2 |
| chrono-native-nsc | 5 | 0% | 20% | 80% | 0.162 | 0.173 | 0 |
| drake-gonthier | 5 | 20% | 60% | 100% | 0.0387 | 0.0311 | 0 |
| drake-gonthier-thesis | 5 | 0% | 40% | 80% | 0.0905 | 0.072 | 0 |
| drake-distributed | 5 | 80% | 100% | 100% | 0.00073 | 0.00182 | 0 |
| drake-distributed-vertex | 5 | 40% | 60% | 80% | 0.016 | 0.0191 | 0 |
| drake-distributed-centroid | 5 | 40% | 80% | 100% | 0.016 | 0.00954 | 0 |
| drake-pcm | 5 | 40% | 60% | 100% | 0.0216 | 0.0124 | 0 |
| drake-native | 5 | 80% | 100% | 100% | 0.00326 | 0.00493 | 0 |
| drake-native-point | 5 | 0% | 0% | 40% | 0.222 | 0.242 | 0 |
| mujoco-gonthier | 5 | 20% | 60% | 100% | 0.0387 | 0.0311 | 0 |
| mujoco-gonthier-thesis | 5 | 0% | 40% | 80% | 0.0905 | 0.0724 | 0 |
| mujoco-distributed | 5 | 80% | 100% | 100% | 0.000745 | 0.00182 | 0 |
| mujoco-distributed-vertex | 5 | 40% | 60% | 80% | 0.016 | 0.0192 | 0 |
| mujoco-distributed-centroid | 5 | 40% | 80% | 100% | 0.016 | 0.00952 | 0 |
| mujoco-pcm | 5 | 40% | 60% | 100% | 0.0217 | 0.0123 | 0 |
| mujoco-native | 5 | 0% | 0% | 40% | 0.329 | 0.288 | 0 |
| mujoco-native-default | 5 | 0% | 0% | 40% | 0.242 | 0.241 | 0 |

### Class C

| method | n | ≤1 % | ≤5 % | ≤20 % | median | geo-mean | failed |
|---|---|---|---|---|---|---|---|
| chrono-gonthier | 7 | 57% | 86% | 100% | 0.00948 | 0.00812 | 0 |
| chrono-gonthier-thesis | 7 | 43% | 57% | 57% | 0.0305 | 0.0949 | 0 |
| chrono-distributed | 7 | 57% | 100% | 100% | 0.00948 | 0.00838 | 0 |
| chrono-distributed-vertex | 7 | 57% | 100% | 100% | 0.00948 | 0.00838 | 0 |
| chrono-distributed-centroid | 7 | 57% | 100% | 100% | 0.00948 | 0.00838 | 0 |
| chrono-pcm | 7 | 29% | 43% | 43% | 0.531 | 0.357 | 0 |
| drake-gonthier | 7 | 57% | 86% | 100% | 0.00948 | 0.00812 | 0 |
| drake-gonthier-thesis | 7 | 43% | 57% | 57% | 0.0305 | 0.117 | 0 |
| drake-distributed | 7 | 57% | 100% | 100% | 0.00948 | 0.00838 | 0 |
| drake-distributed-vertex | 7 | 57% | 100% | 100% | 0.00948 | 0.00838 | 0 |
| drake-distributed-centroid | 7 | 57% | 100% | 100% | 0.00948 | 0.00838 | 0 |
| drake-pcm | 7 | 29% | 43% | 43% | 0.531 | 0.357 | 0 |
| drake-native | 7 | 14% | 29% | 71% | 0.114 | 0.0834 | 0 |
| mujoco-gonthier | 7 | 57% | 86% | 100% | 0.00948 | 0.00812 | 0 |
| mujoco-gonthier-thesis | 7 | 43% | 57% | 57% | 0.0305 | 0.0945 | 0 |
| mujoco-distributed | 7 | 57% | 100% | 100% | 0.00948 | 0.00838 | 0 |
| mujoco-distributed-vertex | 7 | 57% | 100% | 100% | 0.00948 | 0.00838 | 0 |
| mujoco-distributed-centroid | 7 | 57% | 100% | 100% | 0.00948 | 0.00838 | 0 |
| mujoco-pcm | 7 | 29% | 43% | 43% | 0.531 | 0.357 | 0 |

## Paired error ratios, uncapped

Ratio = error of the first over the second (below 1 favours the first), geometric mean over scenarios, 95 % interval from a stratified bootstrap (scenarios resampled within class).

| pair | subset | n | excl | ratio | 95 % CI | wins/losses |
|---|---|---|---|---|---|---|
| chrono-distributed / chrono-gonthier | A | 21 | 0 | 0.8 | 0.484 – 1.2 | 11/8 |
| chrono-distributed / chrono-gonthier | A+B | 26 | 0 | 0.484 | 0.303 – 0.738 | 16/8 |
| chrono-distributed / chrono-gonthier | C | 7 | 0 | 1.03 | 0.843 – 1.3 | 4/1 |
| chrono-gonthier-thesis / chrono-gonthier | A | 21 | 0 | 5.14 | 1.73 – 20.6 | 6/13 |
| chrono-gonthier-thesis / chrono-gonthier | A+B | 26 | 0 | 4.41 | 1.84 – 13.4 | 7/17 |
| chrono-gonthier-thesis / chrono-gonthier | C | 7 | 0 | 11.7 | 2.49 – 85.7 | 2/4 |
| chrono-distributed / chrono-gonthier-thesis | A | 21 | 0 | 0.156 | 0.0422 – 0.441 | 16/5 |
| chrono-distributed / chrono-gonthier-thesis | A+B | 26 | 0 | 0.11 | 0.0353 – 0.279 | 20/6 |
| chrono-distributed / chrono-gonthier-thesis | C | 7 | 0 | 0.0883 | 0.0117 – 0.508 | 5/2 |
| chrono-distributed / chrono-distributed-vertex | A | 21 | 0 | 0.567 | 0.301 – 0.997 | 7/10 |
| chrono-distributed / chrono-distributed-vertex | A+B | 26 | 0 | 0.402 | 0.206 – 0.721 | 12/10 |
| chrono-distributed / chrono-distributed-vertex | C | 7 | 0 | 1 | 1 – 1 | 1/2 |
| chrono-distributed / chrono-distributed-centroid | A | 21 | 0 | 0.502 | 0.238 – 1 | 7/10 |
| chrono-distributed / chrono-distributed-centroid | A+B | 26 | 0 | 0.417 | 0.205 – 0.818 | 10/10 |
| chrono-distributed / chrono-distributed-centroid | C | 7 | 0 | 1 | 1 – 1 | 2/1 |
| chrono-distributed / chrono-pcm | A | 21 | 0 | 0.125 | 0.031 – 0.405 | 17/4 |
| chrono-distributed / chrono-pcm | A+B | 26 | 0 | 0.129 | 0.0391 – 0.366 | 21/5 |
| chrono-distributed / chrono-pcm | C | 7 | 0 | 0.0235 | 0.00153 – 0.247 | 6/1 |
| chrono-gonthier / chrono-native | A | 21 | 0 | 0.708 | 0.16 – 3.42 | 12/9 |
| chrono-gonthier / chrono-native | A+B | 24 | 2 | 0.624 | 0.172 – 2.49 | 15/9 |
| chrono-gonthier-thesis / chrono-native | A | 21 | 0 | 3.64 | 1.29 – 12.3 | 8/13 |
| chrono-gonthier-thesis / chrono-native | A+B | 24 | 2 | 2.84 | 1.11 – 8.15 | 10/14 |
| chrono-distributed / chrono-native | A | 21 | 0 | 0.567 | 0.13 – 2.73 | 13/8 |
| chrono-distributed / chrono-native | A+B | 24 | 2 | 0.402 | 0.111 – 1.65 | 16/8 |
| chrono-distributed-vertex / chrono-native | A | 21 | 0 | 0.999 | 0.246 – 4.32 | 11/10 |
| chrono-distributed-vertex / chrono-native | A+B | 24 | 2 | 0.698 | 0.211 – 2.47 | 14/10 |
| chrono-distributed-centroid / chrono-native | A | 21 | 0 | 1.13 | 0.291 – 5 | 10/11 |
| chrono-distributed-centroid / chrono-native | A+B | 24 | 2 | 0.734 | 0.213 – 2.66 | 13/11 |
| chrono-pcm / chrono-native | A | 21 | 0 | 4.54 | 1.51 – 17.8 | 6/15 |
| chrono-pcm / chrono-native | A+B | 24 | 2 | 2.59 | 0.956 – 8.76 | 9/15 |
| chrono-gonthier / chrono-native-nsc | A | 20 | 1 | 1.02 | 0.193 – 8.25 | 10/7 |
| chrono-gonthier / chrono-native-nsc | A+B | 25 | 1 | 0.72 | 0.183 – 4.05 | 14/8 |
| chrono-gonthier-thesis / chrono-native-nsc | A | 20 | 1 | 5.68 | 0.683 – 66.5 | 10/10 |
| chrono-gonthier-thesis / chrono-native-nsc | A+B | 25 | 1 | 3.37 | 0.623 – 24.2 | 13/12 |
| chrono-distributed / chrono-native-nsc | A | 20 | 1 | 0.827 | 0.129 – 7.24 | 10/8 |
| chrono-distributed / chrono-native-nsc | A+B | 25 | 1 | 0.346 | 0.0751 – 1.95 | 15/8 |
| chrono-distributed-vertex / chrono-native-nsc | A | 20 | 1 | 1.5 | 0.342 – 11.3 | 10/8 |
| chrono-distributed-vertex / chrono-native-nsc | A+B | 25 | 1 | 0.892 | 0.244 – 4.66 | 13/10 |
| chrono-distributed-centroid / chrono-native-nsc | A | 20 | 1 | 1.71 | 0.412 – 12.9 | 10/8 |
| chrono-distributed-centroid / chrono-native-nsc | A+B | 25 | 1 | 0.859 | 0.255 – 4.22 | 15/8 |
| chrono-pcm / chrono-native-nsc | A | 20 | 1 | 7.38 | 0.864 – 95.9 | 10/10 |
| chrono-pcm / chrono-native-nsc | A+B | 25 | 1 | 2.92 | 0.491 – 23.4 | 14/11 |
| drake-distributed / drake-gonthier | A | 21 | 0 | 0.799 | 0.49 – 1.19 | 10/9 |
| drake-distributed / drake-gonthier | A+B | 26 | 0 | 0.483 | 0.303 – 0.751 | 15/9 |
| drake-distributed / drake-gonthier | C | 7 | 0 | 1.03 | 0.843 – 1.3 | 4/1 |
| drake-gonthier-thesis / drake-gonthier | A | 20 | 1 | 5.48 | 1.76 – 21.9 | 5/13 |
| drake-gonthier-thesis / drake-gonthier | A+B | 25 | 1 | 4.61 | 1.85 – 14.4 | 6/17 |
| drake-gonthier-thesis / drake-gonthier | C | 7 | 0 | 14.4 | 2.5 – 158 | 2/4 |
| drake-distributed / drake-gonthier-thesis | A | 20 | 1 | 0.144 | 0.0362 – 0.432 | 16/4 |
| drake-distributed / drake-gonthier-thesis | A+B | 25 | 1 | 0.102 | 0.0326 – 0.273 | 20/5 |
| drake-distributed / drake-gonthier-thesis | C | 7 | 0 | 0.0719 | 0.00642 – 0.505 | 5/2 |
| drake-distributed / drake-distributed-vertex | A | 21 | 0 | 0.567 | 0.291 – 0.998 | 7/10 |
| drake-distributed / drake-distributed-vertex | A+B | 26 | 0 | 0.402 | 0.203 – 0.727 | 12/10 |
| drake-distributed / drake-distributed-vertex | C | 7 | 0 | 1 | 1 – 1 | 2/1 |
| drake-distributed / drake-distributed-centroid | A | 21 | 0 | 0.505 | 0.238 – 1 | 7/10 |
| drake-distributed / drake-distributed-centroid | A+B | 26 | 0 | 0.418 | 0.203 – 0.741 | 10/10 |
| drake-distributed / drake-distributed-centroid | C | 7 | 0 | 1 | 1 – 1 | 3/0 |
| drake-distributed / drake-pcm | A | 21 | 0 | 0.127 | 0.0325 – 0.408 | 17/4 |
| drake-distributed / drake-pcm | A+B | 26 | 0 | 0.13 | 0.041 – 0.375 | 21/5 |
| drake-distributed / drake-pcm | C | 7 | 0 | 0.0235 | 0.00146 – 0.266 | 6/1 |
| drake-gonthier / drake-native | A | 20 | 1 | 0.714 | 0.194 – 5.23 | 16/4 |
| drake-gonthier / drake-native | A+B | 25 | 1 | 1.1 | 0.374 – 5.58 | 16/9 |
| drake-gonthier / drake-native | C | 7 | 0 | 0.0973 | 0.0314 – 0.314 | 6/1 |
| drake-gonthier-thesis / drake-native | A | 19 | 2 | 1.04 | 0.521 – 2.15 | 11/8 |
| drake-gonthier-thesis / drake-native | A+B | 24 | 2 | 1.8 | 0.995 – 3.38 | 11/13 |
| drake-gonthier-thesis / drake-native | C | 7 | 0 | 1.4 | 0.078 – 28.2 | 3/4 |
| drake-distributed / drake-native | A | 20 | 1 | 0.564 | 0.159 – 3.9 | 17/3 |
| drake-distributed / drake-native | A+B | 25 | 1 | 0.518 | 0.18 – 2.49 | 21/4 |
| drake-distributed / drake-native | C | 7 | 0 | 0.1 | 0.0339 – 0.305 | 6/1 |
| drake-distributed-vertex / drake-native | A | 20 | 1 | 1.02 | 0.247 – 7.82 | 14/6 |
| drake-distributed-vertex / drake-native | A+B | 25 | 1 | 1.34 | 0.381 – 7.16 | 16/9 |
| drake-distributed-vertex / drake-native | C | 7 | 0 | 0.1 | 0.0342 – 0.306 | 6/1 |
| drake-distributed-centroid / drake-native | A | 20 | 1 | 1.16 | 0.265 – 8.73 | 14/6 |
| drake-distributed-centroid / drake-native | A+B | 25 | 1 | 1.28 | 0.347 – 6.86 | 16/9 |
| drake-distributed-centroid / drake-native | C | 7 | 0 | 0.1 | 0.0345 – 0.305 | 6/1 |
| drake-pcm / drake-native | A | 20 | 1 | 2.86 | 0.586 – 25.3 | 11/9 |
| drake-pcm / drake-native | A+B | 25 | 1 | 2.78 | 0.749 – 16.1 | 14/11 |
| drake-pcm / drake-native | C | 7 | 0 | 4.28 | 0.196 – 121 | 3/4 |
| drake-gonthier / drake-native-point | A | 21 | 0 | 0.931 | 0.203 – 3.15 | 10/10 |
| drake-gonthier / drake-native-point | A+B | 26 | 0 | 0.636 | 0.186 – 1.8 | 15/10 |
| drake-gonthier-thesis / drake-native-point | A | 20 | 1 | 4.05 | 1.47 – 10.3 | 5/15 |
| drake-gonthier-thesis / drake-native-point | A+B | 25 | 1 | 2.4 | 1.04 – 5.21 | 9/16 |
| drake-distributed / drake-native-point | A | 21 | 0 | 0.743 | 0.166 – 2.54 | 10/10 |
| drake-distributed / drake-native-point | A+B | 26 | 0 | 0.307 | 0.0886 – 0.842 | 15/10 |
| drake-distributed-vertex / drake-native-point | A | 21 | 0 | 1.31 | 0.248 – 4.87 | 7/13 |
| drake-distributed-vertex / drake-native-point | A+B | 26 | 0 | 0.764 | 0.197 – 2.42 | 11/14 |
| drake-distributed-centroid / drake-native-point | A | 21 | 0 | 1.47 | 0.29 – 5.97 | 7/13 |
| drake-distributed-centroid / drake-native-point | A+B | 26 | 0 | 0.734 | 0.188 – 2.4 | 12/13 |
| drake-pcm / drake-native-point | A | 21 | 0 | 5.87 | 1.76 – 18.3 | 4/17 |
| drake-pcm / drake-native-point | A+B | 26 | 0 | 2.36 | 0.878 – 6.08 | 9/17 |
| mujoco-distributed / mujoco-gonthier | A | 21 | 0 | 0.8 | 0.49 – 1.2 | 11/8 |
| mujoco-distributed / mujoco-gonthier | A+B | 26 | 0 | 0.484 | 0.301 – 0.752 | 16/8 |
| mujoco-distributed / mujoco-gonthier | C | 7 | 0 | 1.03 | 0.843 – 1.3 | 4/1 |
| mujoco-gonthier-thesis / mujoco-gonthier | A | 20 | 1 | 5.58 | 1.77 – 22.4 | 5/13 |
| mujoco-gonthier-thesis / mujoco-gonthier | A+B | 25 | 1 | 4.69 | 1.85 – 14.2 | 6/17 |
| mujoco-gonthier-thesis / mujoco-gonthier | C | 7 | 0 | 11.6 | 2.5 – 85.5 | 2/4 |
| mujoco-distributed / mujoco-gonthier-thesis | A | 20 | 1 | 0.142 | 0.0375 – 0.434 | 16/4 |
| mujoco-distributed / mujoco-gonthier-thesis | A+B | 25 | 1 | 0.1 | 0.0333 – 0.262 | 20/5 |
| mujoco-distributed / mujoco-gonthier-thesis | C | 7 | 0 | 0.0886 | 0.0119 – 0.48 | 5/2 |
| mujoco-distributed / mujoco-distributed-vertex | A | 21 | 0 | 0.567 | 0.301 – 0.998 | 8/9 |
| mujoco-distributed / mujoco-distributed-vertex | A+B | 26 | 0 | 0.402 | 0.214 – 0.735 | 13/9 |
| mujoco-distributed / mujoco-distributed-vertex | C | 7 | 0 | 1 | 1 – 1 | 1/2 |
| mujoco-distributed / mujoco-distributed-centroid | A | 21 | 0 | 0.502 | 0.238 – 1 | 7/10 |
| mujoco-distributed / mujoco-distributed-centroid | A+B | 26 | 0 | 0.417 | 0.203 – 0.818 | 10/10 |
| mujoco-distributed / mujoco-distributed-centroid | C | 7 | 0 | 1 | 1 – 1 | 2/1 |
| mujoco-distributed / mujoco-pcm | A | 21 | 0 | 0.125 | 0.0322 – 0.402 | 17/4 |
| mujoco-distributed / mujoco-pcm | A+B | 26 | 0 | 0.129 | 0.04 – 0.371 | 21/5 |
| mujoco-distributed / mujoco-pcm | C | 7 | 0 | 0.0235 | 0.00146 – 0.247 | 6/1 |
| mujoco-gonthier / mujoco-native | A | 21 | 0 | 1.19 | 0.292 – 7.8 | 13/8 |
| mujoco-gonthier / mujoco-native | A+B | 26 | 0 | 0.752 | 0.237 – 3.54 | 18/8 |
| mujoco-gonthier-thesis / mujoco-native | A | 20 | 1 | 3.07 | 0.842 – 15.4 | 9/11 |
| mujoco-gonthier-thesis / mujoco-native | A+B | 25 | 1 | 1.86 | 0.641 – 6.7 | 12/13 |
| mujoco-distributed / mujoco-native | A | 21 | 0 | 0.955 | 0.23 – 6.61 | 14/7 |
| mujoco-distributed / mujoco-native | A+B | 26 | 0 | 0.364 | 0.112 – 1.77 | 19/7 |
| mujoco-distributed-vertex / mujoco-native | A | 21 | 0 | 1.68 | 0.362 – 11.7 | 11/10 |
| mujoco-distributed-vertex / mujoco-native | A+B | 26 | 0 | 0.905 | 0.243 – 4.47 | 14/12 |
| mujoco-distributed-centroid / mujoco-native | A | 21 | 0 | 1.9 | 0.388 – 13.2 | 11/10 |
| mujoco-distributed-centroid / mujoco-native | A+B | 26 | 0 | 0.873 | 0.231 – 4.43 | 16/10 |
| mujoco-pcm / mujoco-native | A | 21 | 0 | 7.62 | 1.13 – 77 | 9/12 |
| mujoco-pcm / mujoco-native | A+B | 26 | 0 | 2.81 | 0.577 – 18.5 | 14/12 |
| mujoco-gonthier / mujoco-native-default | A | 21 | 0 | 0.347 | 0.0739 – 2.41 | 15/6 |
| mujoco-gonthier / mujoco-native-default | A+B | 26 | 0 | 0.287 | 0.0797 – 1.42 | 19/7 |
| mujoco-gonthier-thesis / mujoco-native-default | A | 20 | 1 | 0.928 | 0.312 – 3.17 | 12/8 |
| mujoco-gonthier-thesis / mujoco-native-default | A+B | 25 | 1 | 0.74 | 0.291 – 2.02 | 15/10 |
| mujoco-distributed / mujoco-native-default | A | 21 | 0 | 0.278 | 0.0558 – 2.06 | 15/6 |
| mujoco-distributed / mujoco-native-default | A+B | 26 | 0 | 0.139 | 0.0363 – 0.761 | 20/6 |
| mujoco-distributed-vertex / mujoco-native-default | A | 21 | 0 | 0.49 | 0.115 – 3.04 | 15/6 |
| mujoco-distributed-vertex / mujoco-native-default | A+B | 26 | 0 | 0.346 | 0.095 – 1.66 | 18/8 |
| mujoco-distributed-centroid / mujoco-native-default | A | 21 | 0 | 0.554 | 0.12 – 3.56 | 15/6 |
| mujoco-distributed-centroid / mujoco-native-default | A+B | 26 | 0 | 0.333 | 0.0955 – 1.51 | 20/6 |
| mujoco-pcm / mujoco-native-default | A | 21 | 0 | 2.22 | 0.468 – 14.9 | 12/9 |
| mujoco-pcm / mujoco-native-default | A+B | 26 | 0 | 1.07 | 0.28 – 5.39 | 16/10 |

## Robust paired ratios

Errors capped at 100%; nan, timeout and failed count as 100% for the method they belong to (no pair is left out); scenarios with a common stem and a number (`pieces*`) are one unit; a ratio within a factor 1.25 of 1 is a tie. `A-hK` is class A without `bowl`, `trough`, `turntable`, `edge`. Leave-one-out: range of the ratio when one unit is removed. The last column is the ratio without the cap and with failed pairs left out, and its leave-one-out range.

### Primary comparisons (p: bootstrap, computed without sampling, Holm-adjusted over this family)

| pair | subset | units | ratio | 95 % CI | p | leave-one-out (unit removed) | W/T/L | sign p | uncapped ratio [leave-one-out] |
|---|---|---|---|---|---|---|---|---|---|
| chrono-distributed / chrono-gonthier | A | 18 | 0.925 | 0.641 – 1.31 | 1 | 0.835 – 1.05 (wedge, energy) | 4/10/4 | 1 | 0.926 [0.835 – 1.05] |
| chrono-distributed / chrono-gonthier | A+B | 23 | 0.507 | 0.343 – 0.766 | 0.0215 | 0.456 – 0.603 (wedge, contensou) | 8/11/4 | 0.388 | 0.508 [0.456 – 0.603] |
| chrono-gonthier / chrono-native | A | 18 | 0.625 | 0.164 – 2.23 | 1 | 0.474 – 0.9 (oblique, push) | 10/0/8 | 0.815 | 0.885 [0.501 – 1.3] |
| chrono-gonthier / chrono-native | A+B | 23 | 0.461 | 0.16 – 1.28 | 1 | 0.367 – 0.603 (oblique, push) | 15/0/8 | 0.21 | 0.743 [0.454 – 1.02] |
| chrono-distributed / chrono-native | A | 18 | 0.578 | 0.17 – 1.83 | 1 | 0.45 – 0.829 (edge, push) | 11/0/7 | 0.481 | 0.819 [0.462 – 1.2] |
| chrono-distributed / chrono-native | A+B | 23 | 0.234 | 0.0849 – 0.619 | 0.0456 | 0.185 – 0.3 (edge, contensou) | 16/0/7 | 0.0931 | 0.525 [0.315 – 0.71] |
| drake-gonthier / drake-native | A | 18 | 0.39 | 0.151 – 1.28 | 0.965 | 0.246 – 0.482 (massratio, wedge) | 15/0/3 | 0.00754 | 0.737 [0.272 – 0.96] |
| drake-gonthier / drake-native | A+B | 23 | 0.715 | 0.329 – 1.85 | 1 | 0.514 – 0.865 (massratio, wedge) | 15/0/8 | 0.21 | 1.2 [0.575 – 1.5] |
| drake-distributed / drake-native | A | 18 | 0.36 | 0.14 – 1.18 | 0.864 | 0.226 – 0.446 (massratio, energy) | 15/1/2 | 0.00235 | 0.678 [0.249 – 0.885] |
| drake-distributed / drake-native | A+B | 23 | 0.362 | 0.164 – 0.956 | 0.453 | 0.253 – 0.427 (massratio, energy) | 19/1/3 | 0.000855 | 0.59 [0.273 – 0.719] |
| mujoco-gonthier / mujoco-native | A | 18 | 0.784 | 0.254 – 2.77 | 1 | 0.515 – 1 (massratio, twoblocks) | 11/0/7 | 0.481 | 1.28 [0.515 – 1.68] |
| mujoco-gonthier / mujoco-native | A+B | 23 | 0.51 | 0.198 – 1.41 | 1 | 0.361 – 0.604 (massratio, twoblocks) | 15/1/7 | 0.134 | 0.746 [0.361 – 0.899] |
| mujoco-distributed / mujoco-native | A | 18 | 0.726 | 0.21 – 2.72 | 1 | 0.474 – 0.959 (massratio, energy) | 11/0/7 | 0.481 | 1.18 [0.474 – 1.6] |
| mujoco-distributed / mujoco-native | A+B | 23 | 0.259 | 0.1 – 0.745 | 0.154 | 0.178 – 0.319 (massratio, cone) | 16/0/7 | 0.0931 | 0.379 [0.178 – 0.476] |

### Exploratory comparisons (p: bootstrap, unadjusted; not tests)

| pair | subset | units | ratio | 95 % CI | p | leave-one-out (unit removed) | W/T/L | sign p | uncapped ratio [leave-one-out] |
|---|---|---|---|---|---|---|---|---|---|
| chrono-distributed / chrono-gonthier | A-hK | 14 | 0.908 | 0.569 – 1.42 | 0.684 | 0.792 – 1.06 (wedge, energy) | 4/6/4 | 1 | 0.909 [0.793 – 1.06] |
| chrono-gonthier-thesis / chrono-gonthier | A | 18 | 3.66 | 1.52 – 10.6 | 0.0002 | 2.65 – 3.95 (push, vgroove) | 0/9/9 | 0.00391 | 5.57 [3.24 – 6.17] |
| chrono-gonthier-thesis / chrono-gonthier | A+B | 23 | 3.31 | 1.66 – 7.56 | 0.0002 | 2.57 – 3.52 (push, bearing) | 0/10/13 | 0.000244 | 4.6 [3 – 4.96] |
| chrono-gonthier-thesis / chrono-gonthier | A-hK | 14 | 5.03 | 1.67 – 18.2 | 0.0002 | 3.38 – 5.7 (push, vgroove) | 0/6/8 | 0.00781 | 8.63 [4.39 – 10.2] |
| chrono-distributed / chrono-gonthier-thesis | A | 18 | 0.253 | 0.0951 – 0.577 | 0.0004 | 0.227 – 0.348 (journal, push) | 11/5/2 | 0.0225 | 0.166 [0.145 – 0.285] |
| chrono-distributed / chrono-gonthier-thesis | A+B | 23 | 0.153 | 0.0672 – 0.339 | 0.0002 | 0.138 – 0.191 (journal, push) | 15/6/2 | 0.00235 | 0.11 [0.0976 – 0.164] |
| chrono-distributed / chrono-gonthier-thesis | A-hK | 14 | 0.181 | 0.056 – 0.508 | 0.0002 | 0.152 – 0.267 (journal, push) | 10/2/2 | 0.0386 | 0.105 [0.0853 – 0.205] |
| chrono-distributed / chrono-distributed-vertex | A | 18 | 0.846 | 0.607 – 1 | 0.24 | 0.838 – 0.998 (wedge, pieces) | 1/17/0 | 1 | 0.846 [0.838 – 0.998] |
| chrono-distributed / chrono-distributed-vertex | A+B | 23 | 0.526 | 0.305 – 0.855 | 0.0002 | 0.511 – 0.659 (wedge, disk) | 5/18/0 | 0.0625 | 0.526 [0.511 – 0.659] |
| chrono-distributed / chrono-distributed-vertex | A-hK | 14 | 0.807 | 0.527 – 1 | 0.215 | 0.794 – 0.998 (wedge, pieces) | 1/13/0 | 1 | 0.807 [0.794 – 0.998] |
| chrono-distributed / chrono-distributed-centroid | A | 18 | 0.818 | 0.547 – 1 | 0.598 | 0.808 – 1 (vgroove, pieces) | 1/17/0 | 1 | 0.818 [0.808 – 1] |
| chrono-distributed / chrono-distributed-centroid | A+B | 23 | 0.596 | 0.358 – 1 | 0.0466 | 0.582 – 0.711 (vgroove, disk) | 3/20/0 | 0.25 | 0.596 [0.582 – 0.711] |
| chrono-distributed / chrono-distributed-centroid | A-hK | 14 | 0.772 | 0.46 – 1 | 0.603 | 0.757 – 1 (vgroove, pieces) | 1/13/0 | 1 | 0.772 [0.757 – 1] |
| chrono-distributed / chrono-pcm | A | 18 | 0.28 | 0.1 – 0.681 | 0.0014 | 0.239 – 0.389 (cylinder, push) | 10/6/2 | 0.0386 | 0.164 [0.135 – 0.28] |
| chrono-distributed / chrono-pcm | A+B | 23 | 0.243 | 0.0973 – 0.587 | 0.002 | 0.206 – 0.311 (bearing, push) | 13/7/3 | 0.0213 | 0.16 [0.133 – 0.242] |
| chrono-distributed / chrono-pcm | A-hK | 14 | 0.201 | 0.057 – 0.612 | 0.0026 | 0.159 – 0.302 (cylinder, push) | 9/3/2 | 0.0654 | 0.101 [0.0757 – 0.196] |
| chrono-gonthier / chrono-native | A-hK | 14 | 0.443 | 0.0915 – 2.01 | 0.296 | 0.3 – 0.696 (oblique, push) | 8/0/6 | 0.791 | 0.698 [0.326 – 1.13] |
| chrono-gonthier-thesis / chrono-native | A | 18 | 2.29 | 1.09 – 5.18 | 0.0262 | 1.87 – 2.68 (oblique, journal) | 6/2/10 | 0.454 | 4.93 [3.09 – 6.03] |
| chrono-gonthier-thesis / chrono-native | A+B | 23 | 1.53 | 0.825 – 2.9 | 0.194 | 1.28 – 1.74 (oblique, bearing) | 9/3/11 | 0.824 | 3.55 [2.35 – 4.29] |
| chrono-gonthier-thesis / chrono-native | A-hK | 14 | 2.23 | 0.956 – 5.46 | 0.0622 | 1.71 – 2.73 (oblique, journal) | 4/2/8 | 0.388 | 6.02 [3.32 – 7.97] |
| chrono-distributed / chrono-native | A-hK | 14 | 0.402 | 0.0972 – 1.58 | 0.212 | 0.282 – 0.627 (oblique, push) | 9/0/5 | 0.424 | 0.634 [0.294 – 1.02] |
| chrono-distributed-vertex / chrono-native | A | 18 | 0.683 | 0.206 – 2.11 | 0.552 | 0.537 – 0.989 (edge, push) | 10/1/7 | 0.629 | 0.968 [0.551 – 1.43] |
| chrono-distributed-vertex / chrono-native | A+B | 23 | 0.445 | 0.172 – 1.12 | 0.084 | 0.362 – 0.581 (edge, push) | 15/1/7 | 0.134 | 0.645 [0.392 – 0.881] |
| chrono-distributed-vertex / chrono-native | A-hK | 14 | 0.499 | 0.121 – 1.86 | 0.324 | 0.355 – 0.79 (oblique, push) | 8/1/5 | 0.581 | 0.786 [0.37 – 1.29] |
| chrono-distributed-centroid / chrono-native | A | 18 | 0.707 | 0.206 – 2.24 | 0.577 | 0.557 – 1.03 (edge, push) | 10/0/8 | 0.815 | 1 [0.571 – 1.48] |
| chrono-distributed-centroid / chrono-native | A+B | 23 | 0.393 | 0.151 – 0.974 | 0.0472 | 0.318 – 0.51 (edge, push) | 15/0/8 | 0.21 | 0.623 [0.378 – 0.85] |
| chrono-distributed-centroid / chrono-native | A-hK | 14 | 0.521 | 0.126 – 2.01 | 0.357 | 0.373 – 0.828 (oblique, push) | 8/0/6 | 0.791 | 0.821 [0.388 – 1.35] |
| chrono-pcm / chrono-native | A | 18 | 2.06 | 0.924 – 4.64 | 0.0746 | 1.71 – 2.45 (edge, cylinder) | 6/2/10 | 0.454 | 5 [2.77 – 6.26] |
| chrono-pcm / chrono-native | A+B | 23 | 0.962 | 0.501 – 1.89 | 0.914 | 0.804 – 1.19 (edge, bearing) | 11/2/10 | 1 | 2.6 [1.52 – 3.44] |
| chrono-pcm / chrono-native | A-hK | 14 | 2 | 0.809 – 4.86 | 0.131 | 1.59 – 2.5 (oblique, cylinder) | 4/2/8 | 0.388 | 6.29 [2.96 – 8.57] |
| chrono-gonthier / chrono-native-nsc | A | 18 | 1 | 0.32 – 3.41 | 0.977 | 0.666 – 1.23 (massratio, pieces) | 8/4/6 | 0.791 | 1.93 [0.755 – 2.55] |
| chrono-gonthier / chrono-native-nsc | A+B | 23 | 0.689 | 0.262 – 1.88 | 0.457 | 0.495 – 0.813 (massratio, bearing) | 12/4/7 | 0.359 | 1.13 [0.537 – 1.37] |
| chrono-gonthier / chrono-native-nsc | A-hK | 14 | 0.958 | 0.237 – 4.67 | 0.932 | 0.561 – 1.26 (massratio, pieces) | 6/3/5 | 1 | 2.26 [0.655 – 3.31] |
| chrono-gonthier-thesis / chrono-native-nsc | A | 18 | 3.66 | 0.817 – 18.1 | 0.11 | 2.63 – 4.87 (massratio, journal) | 8/1/9 | 1 | 11.9 [5.21 – 17.6] |
| chrono-gonthier-thesis / chrono-native-nsc | A+B | 23 | 2.28 | 0.646 – 8.57 | 0.209 | 1.73 – 2.86 (massratio, bearing) | 10/2/11 | 1 | 5.55 [2.86 – 7.33] |
| chrono-gonthier-thesis / chrono-native-nsc | A-hK | 14 | 4.82 | 0.718 – 34 | 0.109 | 3.2 – 7.16 (massratio, journal) | 6/0/8 | 0.791 | 23 [8.09 – 40.9] |
| chrono-distributed / chrono-native-nsc | A | 18 | 0.925 | 0.265 – 3.46 | 0.89 | 0.613 – 1.21 (massratio, pieces) | 8/3/7 | 1 | 1.83 [0.716 – 2.56] |
| chrono-distributed / chrono-native-nsc | A+B | 23 | 0.35 | 0.131 – 0.992 | 0.0488 | 0.243 – 0.42 (massratio, ring) | 13/3/7 | 0.263 | 0.568 [0.262 – 0.704] |
| chrono-distributed / chrono-native-nsc | A-hK | 14 | 0.87 | 0.189 – 4.48 | 0.842 | 0.506 – 1.23 (massratio, pieces) | 6/2/6 | 1 | 2.12 [0.612 – 3.34] |
| chrono-distributed-vertex / chrono-native-nsc | A | 18 | 1.09 | 0.341 – 3.69 | 0.92 | 0.732 – 1.43 (massratio, energy) | 8/3/7 | 1 | 2.19 [0.863 – 3.05] |
| chrono-distributed-vertex / chrono-native-nsc | A+B | 23 | 0.665 | 0.236 – 1.93 | 0.441 | 0.477 – 0.807 (massratio, ring) | 11/3/9 | 0.824 | 1.11 [0.53 – 1.4] |
| chrono-distributed-vertex / chrono-native-nsc | A-hK | 14 | 1.08 | 0.263 – 4.88 | 0.931 | 0.637 – 1.53 (massratio, energy) | 6/2/6 | 1 | 2.67 [0.785 – 4.22] |
| chrono-distributed-centroid / chrono-native-nsc | A | 18 | 1.13 | 0.354 – 3.85 | 0.841 | 0.759 – 1.48 (massratio, energy) | 8/3/7 | 1 | 2.27 [0.897 – 3.17] |
| chrono-distributed-centroid / chrono-native-nsc | A+B | 23 | 0.587 | 0.212 – 1.69 | 0.305 | 0.418 – 0.722 (massratio, ring) | 12/4/7 | 0.359 | 0.976 [0.462 – 1.24] |
| chrono-distributed-centroid / chrono-native-nsc | A-hK | 14 | 1.13 | 0.277 – 5.04 | 0.894 | 0.668 – 1.61 (massratio, energy) | 6/2/6 | 1 | 2.8 [0.827 – 4.44] |
| chrono-pcm / chrono-native-nsc | A | 18 | 3.3 | 0.659 – 16.8 | 0.148 | 2.36 – 4.57 (massratio, energy) | 8/1/9 | 1 | 12.5 [4.82 – 19.2] |
| chrono-pcm / chrono-native-nsc | A+B | 23 | 1.44 | 0.363 – 5.79 | 0.593 | 1.07 – 1.95 (massratio, bearing) | 12/1/10 | 0.832 | 3.87 [1.77 – 5.58] |
| chrono-pcm / chrono-native-nsc | A-hK | 14 | 4.32 | 0.552 – 34 | 0.171 | 2.84 – 6.76 (massratio, energy) | 6/0/8 | 0.791 | 25.2 [7.5 – 47.3] |
| drake-distributed / drake-gonthier | A | 18 | 0.923 | 0.64 – 1.3 | 0.679 | 0.832 – 1.04 (wedge, energy) | 4/10/4 | 1 | 0.924 [0.833 – 1.05] |
| drake-distributed / drake-gonthier | A+B | 23 | 0.506 | 0.338 – 0.774 | 0.002 | 0.455 – 0.602 (wedge, contensou) | 8/11/4 | 0.388 | 0.507 [0.455 – 0.602] |
| drake-distributed / drake-gonthier | A-hK | 14 | 0.905 | 0.569 – 1.41 | 0.675 | 0.79 – 1.06 (wedge, energy) | 4/6/4 | 1 | 0.906 [0.79 – 1.06] |
| drake-gonthier-thesis / drake-gonthier | A | 18 | 3.58 | 1.48 – 10.5 | 0.0002 | 2.59 – 3.87 (push, vgroove) | 0/10/8 | 0.00781 | 6.03 [3.41 – 6.76] |
| drake-gonthier-thesis / drake-gonthier | A+B | 23 | 3.26 | 1.61 – 7.58 | 0.0002 | 2.53 – 3.46 (push, bearing) | 0/11/12 | 0.000488 | 4.85 [3.11 – 5.26] |
| drake-gonthier-thesis / drake-gonthier | A-hK | 14 | 4.9 | 1.63 – 17.6 | 0.0002 | 3.28 – 5.54 (push, vgroove) | 0/7/7 | 0.0156 | 9.9 [4.82 – 12] |
| drake-distributed / drake-gonthier-thesis | A | 18 | 0.258 | 0.0943 – 0.59 | 0.0002 | 0.231 – 0.354 (journal, push) | 10/6/2 | 0.0386 | 0.152 [0.131 – 0.268] |
| drake-distributed / drake-gonthier-thesis | A+B | 23 | 0.155 | 0.0678 – 0.342 | 0.0002 | 0.14 – 0.194 (journal, push) | 14/7/2 | 0.00418 | 0.101 [0.0886 – 0.153] |
| drake-distributed / drake-gonthier-thesis | A-hK | 14 | 0.185 | 0.0571 – 0.521 | 0.0002 | 0.156 – 0.274 (journal, push) | 9/3/2 | 0.0654 | 0.0907 [0.0712 – 0.185] |
| drake-distributed / drake-distributed-vertex | A | 18 | 0.847 | 0.608 – 1 | 0.245 | 0.838 – 0.999 (block, pieces) | 1/17/0 | 1 | 0.847 [0.838 – 0.999] |
| drake-distributed / drake-distributed-vertex | A+B | 23 | 0.526 | 0.305 – 0.858 | 0.0006 | 0.511 – 0.66 (block, disk) | 5/18/0 | 0.0625 | 0.526 [0.511 – 0.66] |
| drake-distributed / drake-distributed-vertex | A-hK | 14 | 0.808 | 0.528 – 1 | 0.226 | 0.794 – 0.998 (block, pieces) | 1/13/0 | 1 | 0.808 [0.794 – 0.998] |
| drake-distributed / drake-distributed-centroid | A | 18 | 0.823 | 0.547 – 1.02 | 0.654 | 0.808 – 1.01 (block, pieces) | 1/17/0 | 1 | 0.823 [0.808 – 1.01] |
| drake-distributed / drake-distributed-centroid | A+B | 23 | 0.599 | 0.357 – 1 | 0.0546 | 0.582 – 0.714 (block, disk) | 3/20/0 | 0.25 | 0.599 [0.582 – 0.714] |
| drake-distributed / drake-distributed-centroid | A-hK | 14 | 0.778 | 0.461 – 1.02 | 0.664 | 0.757 – 1.01 (block, pieces) | 1/13/0 | 1 | 0.778 [0.757 – 1.01] |
| drake-distributed / drake-pcm | A | 18 | 0.285 | 0.0994 – 0.691 | 0.0032 | 0.244 – 0.397 (cylinder, push) | 9/7/2 | 0.0654 | 0.167 [0.138 – 0.286] |
| drake-distributed / drake-pcm | A+B | 23 | 0.247 | 0.0985 – 0.593 | 0.0008 | 0.209 – 0.317 (bearing, push) | 12/8/3 | 0.0352 | 0.162 [0.135 – 0.246] |
| drake-distributed / drake-pcm | A-hK | 14 | 0.206 | 0.06 – 0.631 | 0.0026 | 0.163 – 0.309 (cylinder, push) | 8/4/2 | 0.109 | 0.103 [0.0776 – 0.201] |
| drake-gonthier / drake-native | A-hK | 14 | 0.351 | 0.107 – 1.58 | 0.161 | 0.191 – 0.459 (massratio, wedge) | 11/0/3 | 0.0574 | 0.823 [0.22 – 1.18] |
| drake-gonthier-thesis / drake-native | A | 18 | 1.4 | 0.568 – 4.12 | 0.529 | 0.95 – 1.65 (massratio, energy) | 10/1/7 | 0.629 | 0.926 [0.708 – 1.09] |
| drake-gonthier-thesis / drake-native | A+B | 23 | 2.33 | 1.1 – 5.69 | 0.0242 | 1.77 – 2.71 (massratio, energy) | 10/1/12 | 0.832 | 1.79 [1.46 – 2.08] |
| drake-gonthier-thesis / drake-native | A-hK | 14 | 1.72 | 0.553 – 6.83 | 0.414 | 1.05 – 2.17 (massratio, energy) | 6/1/7 | 1 | 1.06 [0.743 – 1.33] |
| drake-distributed / drake-native | A-hK | 14 | 0.318 | 0.0977 – 1.5 | 0.129 | 0.171 – 0.417 (massratio, energy) | 11/1/2 | 0.0225 | 0.74 [0.196 – 1.06] |
| drake-distributed-vertex / drake-native | A | 18 | 0.425 | 0.159 – 1.42 | 0.156 | 0.27 – 0.532 (massratio, energy) | 14/1/3 | 0.0127 | 0.809 [0.3 – 1.07] |
| drake-distributed-vertex / drake-native | A+B | 23 | 0.688 | 0.27 – 1.94 | 0.451 | 0.494 – 0.836 (massratio, energy) | 16/1/6 | 0.0525 | 1.15 [0.551 – 1.45] |
| drake-distributed-vertex / drake-native | A-hK | 14 | 0.394 | 0.113 – 1.9 | 0.218 | 0.215 – 0.524 (massratio, energy) | 10/1/3 | 0.0923 | 0.932 [0.251 – 1.37] |
| drake-distributed-centroid / drake-native | A | 18 | 0.438 | 0.156 – 1.5 | 0.154 | 0.278 – 0.549 (massratio, energy) | 14/1/3 | 0.0127 | 0.833 [0.309 – 1.1] |
| drake-distributed-centroid / drake-native | A+B | 23 | 0.605 | 0.234 – 1.74 | 0.317 | 0.432 – 0.731 (massratio, energy) | 16/1/6 | 0.0525 | 1.01 [0.479 – 1.26] |
| drake-distributed-centroid / drake-native | A-hK | 14 | 0.408 | 0.112 – 1.91 | 0.231 | 0.224 – 0.546 (massratio, energy) | 10/1/3 | 0.0923 | 0.969 [0.262 – 1.43] |
| drake-pcm / drake-native | A | 18 | 1.26 | 0.434 – 4.08 | 0.707 | 0.853 – 1.67 (massratio, energy) | 9/3/6 | 0.607 | 2.37 [0.826 – 3.31] |
| drake-pcm / drake-native | A+B | 23 | 1.47 | 0.583 – 3.92 | 0.448 | 1.09 – 1.83 (massratio, energy) | 12/3/8 | 0.503 | 2.4 [1.08 – 3.1] |
| drake-pcm / drake-native | A-hK | 14 | 1.54 | 0.382 – 6.53 | 0.563 | 0.938 – 2.25 (massratio, energy) | 6/2/6 | 1 | 3.68 [0.934 – 5.95] |
| drake-gonthier / drake-native-point | A | 18 | 0.999 | 0.381 – 2.54 | 0.987 | 0.81 – 1.25 (edge, vgroove) | 7/2/9 | 0.804 | 0.835 [0.633 – 1.59] |
| drake-gonthier / drake-native-point | A+B | 23 | 0.64 | 0.296 – 1.34 | 0.254 | 0.533 – 0.744 (edge, vgroove) | 12/2/9 | 0.664 | 0.556 [0.441 – 0.897] |
| drake-gonthier / drake-native-point | A-hK | 14 | 0.655 | 0.227 – 1.83 | 0.442 | 0.522 – 0.848 (oblique, vgroove) | 6/2/6 | 1 | 0.52 [0.349 – 1.16] |
| drake-gonthier-thesis / drake-native-point | A | 18 | 3.58 | 1.21 – 9.7 | 0.0204 | 2.87 – 4.83 (wedge, vgroove) | 3/2/13 | 0.0213 | 3.81 [3.03 – 5.26] |
| drake-gonthier-thesis / drake-native-point | A+B | 23 | 2.08 | 0.884 – 4.68 | 0.104 | 1.72 – 2.56 (wedge, vgroove) | 7/2/14 | 0.189 | 2.14 [1.74 – 2.65] |
| drake-gonthier-thesis / drake-native-point | A-hK | 14 | 3.21 | 0.874 – 11.2 | 0.0922 | 2.39 – 4.7 (wedge, vgroove) | 3/2/9 | 0.146 | 3.46 [2.52 – 5.26] |
| drake-distributed / drake-native-point | A | 18 | 0.922 | 0.333 – 2.44 | 0.883 | 0.744 – 1.18 (edge, energy) | 7/2/9 | 0.804 | 0.771 [0.581 – 1.46] |
| drake-distributed / drake-native-point | A+B | 23 | 0.324 | 0.143 – 0.703 | 0.003 | 0.262 – 0.416 (edge, cone) | 12/2/9 | 0.664 | 0.282 [0.216 – 0.441] |
| drake-distributed / drake-native-point | A-hK | 14 | 0.593 | 0.196 – 1.68 | 0.34 | 0.489 – 0.788 (oblique, energy) | 6/2/6 | 1 | 0.471 [0.314 – 1.05] |
| drake-distributed-vertex / drake-native-point | A | 18 | 1.09 | 0.377 – 2.89 | 0.849 | 0.887 – 1.4 (edge, energy) | 6/2/10 | 0.454 | 0.91 [0.693 – 1.74] |
| drake-distributed-vertex / drake-native-point | A+B | 23 | 0.615 | 0.245 – 1.46 | 0.299 | 0.512 – 0.781 (edge, cone) | 10/2/11 | 1 | 0.535 [0.423 – 0.862] |
| drake-distributed-vertex / drake-native-point | A-hK | 14 | 0.734 | 0.226 – 2.24 | 0.616 | 0.594 – 0.992 (pieces, energy) | 5/2/7 | 0.774 | 0.583 [0.395 – 1.32] |
| drake-distributed-centroid / drake-native-point | A | 18 | 1.12 | 0.385 – 3.03 | 0.843 | 0.915 – 1.45 (edge, energy) | 6/2/10 | 0.454 | 0.937 [0.714 – 1.8] |
| drake-distributed-centroid / drake-native-point | A+B | 23 | 0.541 | 0.218 – 1.32 | 0.183 | 0.448 – 0.711 (edge, cone) | 11/2/10 | 1 | 0.471 [0.37 – 0.754] |
| drake-distributed-centroid / drake-native-point | A-hK | 14 | 0.762 | 0.224 – 2.45 | 0.676 | 0.588 – 1.03 (pieces, energy) | 5/2/7 | 0.774 | 0.605 [0.411 – 1.37] |
| drake-pcm / drake-native-point | A | 18 | 3.23 | 0.997 – 9.72 | 0.045 | 2.64 – 4.39 (wedge, energy) | 3/4/11 | 0.0574 | 4.62 [3.43 – 6.42] |
| drake-pcm / drake-native-point | A+B | 23 | 1.31 | 0.5 – 3.25 | 0.565 | 1.08 – 1.66 (wedge, bearing) | 8/4/11 | 0.648 | 1.74 [1.32 – 2.22] |
| drake-pcm / drake-native-point | A-hK | 14 | 2.88 | 0.703 – 10.8 | 0.152 | 2.19 – 4.26 (wedge, energy) | 3/3/8 | 0.227 | 4.56 [3.08 – 7] |
| mujoco-distributed / mujoco-gonthier | A | 18 | 0.925 | 0.639 – 1.31 | 0.688 | 0.835 – 1.05 (wedge, energy) | 4/10/4 | 1 | 0.926 [0.835 – 1.05] |
| mujoco-distributed / mujoco-gonthier | A+B | 23 | 0.507 | 0.338 – 0.772 | 0.002 | 0.456 – 0.603 (wedge, contensou) | 8/11/4 | 0.388 | 0.508 [0.456 – 0.604] |
| mujoco-distributed / mujoco-gonthier | A-hK | 14 | 0.908 | 0.563 – 1.42 | 0.689 | 0.792 – 1.06 (wedge, energy) | 4/6/4 | 1 | 0.909 [0.793 – 1.06] |
| mujoco-gonthier-thesis / mujoco-gonthier | A | 18 | 3.66 | 1.49 – 10.4 | 0.0002 | 2.65 – 3.95 (push, vgroove) | 0/9/9 | 0.00391 | 6.16 [3.49 – 6.91] |
| mujoco-gonthier-thesis / mujoco-gonthier | A+B | 23 | 3.32 | 1.63 – 7.71 | 0.0002 | 2.57 – 3.52 (push, bearing) | 0/10/13 | 0.000244 | 4.94 [3.17 – 5.36] |
| mujoco-gonthier-thesis / mujoco-gonthier | A-hK | 14 | 5.02 | 1.67 – 18.5 | 0.0002 | 3.38 – 5.7 (push, vgroove) | 0/6/8 | 0.00781 | 10.2 [4.97 – 12.4] |
| mujoco-distributed / mujoco-gonthier-thesis | A | 18 | 0.253 | 0.0971 – 0.58 | 0.0002 | 0.227 – 0.348 (journal, push) | 11/5/2 | 0.0225 | 0.15 [0.129 – 0.263] |
| mujoco-distributed / mujoco-gonthier-thesis | A+B | 23 | 0.153 | 0.0659 – 0.33 | 0.0002 | 0.137 – 0.191 (journal, push) | 15/6/2 | 0.00235 | 0.0997 [0.0872 – 0.15] |
| mujoco-distributed / mujoco-gonthier-thesis | A-hK | 14 | 0.181 | 0.0559 – 0.514 | 0.0002 | 0.152 – 0.267 (journal, push) | 10/2/2 | 0.0386 | 0.0885 [0.0694 – 0.18] |
| mujoco-distributed / mujoco-distributed-vertex | A | 18 | 0.846 | 0.607 – 1 | 0.203 | 0.838 – 0.998 (wedge, pieces) | 1/17/0 | 1 | 0.846 [0.838 – 0.998] |
| mujoco-distributed / mujoco-distributed-vertex | A+B | 23 | 0.526 | 0.311 – 0.869 | 0.0002 | 0.511 – 0.659 (wedge, disk) | 5/18/0 | 0.0625 | 0.526 [0.511 – 0.659] |
| mujoco-distributed / mujoco-distributed-vertex | A-hK | 14 | 0.807 | 0.527 – 1 | 0.177 | 0.794 – 0.998 (wedge, pieces) | 1/13/0 | 1 | 0.807 [0.794 – 0.998] |
| mujoco-distributed / mujoco-distributed-centroid | A | 18 | 0.818 | 0.547 – 1 | 0.585 | 0.808 – 1 (vgroove, pieces) | 1/17/0 | 1 | 0.818 [0.808 – 1] |
| mujoco-distributed / mujoco-distributed-centroid | A+B | 23 | 0.596 | 0.355 – 1 | 0.0458 | 0.582 – 0.711 (vgroove, disk) | 3/20/0 | 0.25 | 0.596 [0.582 – 0.711] |
| mujoco-distributed / mujoco-distributed-centroid | A-hK | 14 | 0.772 | 0.46 – 1 | 0.56 | 0.757 – 1 (vgroove, pieces) | 1/13/0 | 1 | 0.772 [0.757 – 1] |
| mujoco-distributed / mujoco-pcm | A | 18 | 0.282 | 0.0989 – 0.688 | 0.0034 | 0.24 – 0.392 (cylinder, push) | 10/6/2 | 0.0386 | 0.165 [0.136 – 0.282] |
| mujoco-distributed / mujoco-pcm | A+B | 23 | 0.245 | 0.0965 – 0.586 | 0.001 | 0.207 – 0.314 (bearing, push) | 13/7/3 | 0.0213 | 0.161 [0.133 – 0.243] |
| mujoco-distributed / mujoco-pcm | A-hK | 14 | 0.203 | 0.0604 – 0.632 | 0.003 | 0.161 – 0.304 (cylinder, push) | 9/3/2 | 0.0654 | 0.102 [0.0763 – 0.198] |
| mujoco-gonthier / mujoco-native | A-hK | 14 | 0.613 | 0.17 – 2.82 | 0.479 | 0.347 – 0.829 (massratio, twoblocks) | 9/0/5 | 0.424 | 1.15 [0.347 – 1.62] |
| mujoco-gonthier-thesis / mujoco-native | A | 18 | 2.87 | 0.832 – 10.7 | 0.0892 | 2.03 – 3.62 (massratio, energy) | 8/0/10 | 0.815 | 3.17 [1.57 – 4.09] |
| mujoco-gonthier-thesis / mujoco-native | A+B | 23 | 1.69 | 0.624 – 4.98 | 0.319 | 1.26 – 2.08 (massratio, bearing) | 11/1/11 | 1 | 1.78 [1.01 – 2.22] |
| mujoco-gonthier-thesis / mujoco-native | A-hK | 14 | 3.08 | 0.706 – 15.4 | 0.139 | 1.98 – 4.2 (massratio, energy) | 6/0/8 | 0.791 | 3.54 [1.4 – 5] |
| mujoco-distributed / mujoco-native | A-hK | 14 | 0.557 | 0.134 – 2.79 | 0.445 | 0.313 – 0.785 (massratio, energy) | 9/0/5 | 0.424 | 1.04 [0.313 – 1.54] |
| mujoco-distributed-vertex / mujoco-native | A | 18 | 0.857 | 0.243 – 3.43 | 0.807 | 0.566 – 1.14 (massratio, energy) | 10/0/8 | 0.815 | 1.39 [0.566 – 1.91] |
| mujoco-distributed-vertex / mujoco-native | A+B | 23 | 0.492 | 0.171 – 1.54 | 0.204 | 0.348 – 0.6 (massratio, cone) | 13/2/8 | 0.383 | 0.72 [0.348 – 0.894] |
| mujoco-distributed-vertex / mujoco-native | A-hK | 14 | 0.69 | 0.157 – 3.45 | 0.602 | 0.394 – 0.989 (massratio, energy) | 8/0/6 | 0.791 | 1.29 [0.394 – 1.94] |
| mujoco-distributed-centroid / mujoco-native | A | 18 | 0.887 | 0.24 – 3.43 | 0.837 | 0.587 – 1.19 (massratio, energy) | 10/0/8 | 0.815 | 1.44 [0.587 – 1.99] |
| mujoco-distributed-centroid / mujoco-native | A+B | 23 | 0.434 | 0.15 – 1.34 | 0.145 | 0.305 – 0.549 (massratio, cone) | 15/0/8 | 0.21 | 0.635 [0.305 – 0.817] |
| mujoco-distributed-centroid / mujoco-native | A-hK | 14 | 0.721 | 0.161 – 3.63 | 0.657 | 0.413 – 1.04 (massratio, energy) | 8/0/6 | 0.791 | 1.35 [0.413 – 2.03] |
| mujoco-pcm / mujoco-native | A | 18 | 2.57 | 0.662 – 9.98 | 0.179 | 1.81 – 3.63 (massratio, energy) | 8/1/9 | 1 | 7.17 [2.83 – 10.7] |
| mujoco-pcm / mujoco-native | A+B | 23 | 1.06 | 0.339 – 3.36 | 0.958 | 0.774 – 1.41 (massratio, bearing) | 13/1/9 | 0.523 | 2.36 [1.09 – 3.26] |
| mujoco-pcm / mujoco-native | A-hK | 14 | 2.75 | 0.539 – 15.5 | 0.241 | 1.74 – 4.33 (massratio, energy) | 6/1/7 | 1 | 10.2 [3.13 – 17.9] |
| mujoco-gonthier / mujoco-native-default | A | 18 | 0.363 | 0.108 – 1.24 | 0.106 | 0.256 – 0.491 (massratio, push) | 12/0/6 | 0.238 | 0.591 [0.256 – 0.822] |
| mujoco-gonthier / mujoco-native-default | A+B | 23 | 0.29 | 0.108 – 0.786 | 0.0164 | 0.219 – 0.362 (massratio, push) | 16/0/7 | 0.0931 | 0.424 [0.219 – 0.539] |
| mujoco-gonthier / mujoco-native-default | A-hK | 14 | 0.287 | 0.0641 – 1.41 | 0.121 | 0.178 – 0.418 (massratio, push) | 9/0/5 | 0.424 | 0.537 [0.178 – 0.82] |
| mujoco-gonthier-thesis / mujoco-native-default | A | 18 | 1.33 | 0.583 – 3.2 | 0.51 | 1.01 – 1.62 (massratio, pieces) | 8/1/9 | 1 | 1.58 [0.987 – 1.97] |
| mujoco-gonthier-thesis / mujoco-native-default | A+B | 23 | 0.962 | 0.46 – 2.03 | 0.909 | 0.767 – 1.15 (massratio, bearing) | 11/1/11 | 1 | 1.08 [0.743 – 1.32] |
| mujoco-gonthier-thesis / mujoco-native-default | A-hK | 14 | 1.44 | 0.495 – 4.18 | 0.492 | 1.02 – 1.88 (massratio, pieces) | 5/1/8 | 0.581 | 1.82 [0.984 – 2.47] |
| mujoco-distributed / mujoco-native-default | A | 18 | 0.336 | 0.0909 – 1.28 | 0.0982 | 0.236 – 0.453 (massratio, push) | 11/1/6 | 0.332 | 0.547 [0.236 – 0.758] |
| mujoco-distributed / mujoco-native-default | A+B | 23 | 0.147 | 0.053 – 0.407 | 0.0006 | 0.108 – 0.18 (massratio, cone) | 16/1/6 | 0.0525 | 0.215 [0.108 – 0.267] |
| mujoco-distributed / mujoco-native-default | A-hK | 14 | 0.261 | 0.0498 – 1.29 | 0.109 | 0.161 – 0.377 (massratio, push) | 8/1/5 | 0.581 | 0.488 [0.161 – 0.74] |
| mujoco-distributed-vertex / mujoco-native-default | A | 18 | 0.397 | 0.117 – 1.37 | 0.137 | 0.281 – 0.54 (massratio, push) | 11/1/6 | 0.332 | 0.646 [0.281 – 0.904] |
| mujoco-distributed-vertex / mujoco-native-default | A+B | 23 | 0.28 | 0.0942 – 0.849 | 0.0188 | 0.211 – 0.349 (massratio, push) | 14/1/8 | 0.286 | 0.41 [0.211 – 0.52] |
| mujoco-distributed-vertex / mujoco-native-default | A-hK | 14 | 0.323 | 0.0697 – 1.52 | 0.157 | 0.203 – 0.475 (massratio, push) | 8/1/5 | 0.581 | 0.604 [0.203 – 0.932] |
| mujoco-distributed-centroid / mujoco-native-default | A | 18 | 0.411 | 0.121 – 1.43 | 0.16 | 0.292 – 0.56 (massratio, push) | 11/1/6 | 0.332 | 0.669 [0.292 – 0.937] |
| mujoco-distributed-centroid / mujoco-native-default | A+B | 23 | 0.247 | 0.0845 – 0.711 | 0.0106 | 0.185 – 0.309 (massratio, cone) | 15/2/6 | 0.0784 | 0.361 [0.185 – 0.459] |
| mujoco-distributed-centroid / mujoco-native-default | A-hK | 14 | 0.338 | 0.0752 – 1.58 | 0.16 | 0.212 – 0.498 (massratio, push) | 8/1/5 | 0.581 | 0.631 [0.212 – 0.977] |
| mujoco-pcm / mujoco-native-default | A | 18 | 1.19 | 0.476 – 2.92 | 0.712 | 0.901 – 1.59 (massratio, energy) | 8/2/8 | 1 | 3.32 [1.41 – 4.7] |
| mujoco-pcm / mujoco-native-default | A+B | 23 | 0.601 | 0.261 – 1.4 | 0.243 | 0.47 – 0.782 (massratio, bearing) | 12/2/9 | 0.664 | 1.34 [0.663 – 1.81] |
| mujoco-pcm / mujoco-native-default | A-hK | 14 | 1.29 | 0.4 – 4.12 | 0.659 | 0.897 – 1.88 (massratio, energy) | 5/2/7 | 0.774 | 4.8 [1.61 – 7.78] |

## Performance profiles

Share of scenarios where a method is within a factor τ of the best (floored errors). `auc` is the area under the profile over τ = 1…1000 on a log axis.

### A+B

| method | best | τ≤2 | τ≤10 | auc |
|---|---|---|---|---|
| chrono-gonthier | 12% | 35% | 62% | 0.71 |
| chrono-gonthier-thesis | 0% | 15% | 38% | 0.53 |
| chrono-distributed | 23% | 50% | 85% | 0.81 |
| chrono-distributed-vertex | 8% | 27% | 65% | 0.68 |
| chrono-distributed-centroid | 15% | 31% | 65% | 0.69 |
| chrono-pcm | 0% | 27% | 35% | 0.56 |
| chrono-native | 27% | 27% | 42% | 0.57 |
| chrono-native-nsc | 31% | 42% | 50% | 0.60 |
| drake-gonthier | 12% | 35% | 62% | 0.71 |
| drake-gonthier-thesis | 0% | 15% | 38% | 0.53 |
| drake-distributed | 27% | 50% | 85% | 0.81 |
| drake-distributed-vertex | 12% | 27% | 65% | 0.68 |
| drake-distributed-centroid | 19% | 31% | 65% | 0.69 |
| drake-pcm | 0% | 27% | 38% | 0.56 |
| drake-native | 4% | 8% | 54% | 0.65 |
| drake-native-point | 12% | 42% | 58% | 0.65 |
| mujoco-gonthier | 12% | 35% | 62% | 0.71 |
| mujoco-gonthier-thesis | 0% | 15% | 38% | 0.53 |
| mujoco-distributed | 27% | 50% | 85% | 0.81 |
| mujoco-distributed-vertex | 12% | 27% | 65% | 0.68 |
| mujoco-distributed-centroid | 15% | 31% | 65% | 0.69 |
| mujoco-pcm | 4% | 27% | 38% | 0.56 |
| mujoco-native | 15% | 19% | 58% | 0.62 |
| mujoco-native-default | 12% | 12% | 31% | 0.48 |

### C

| method | best | τ≤2 | τ≤10 | auc |
|---|---|---|---|---|
| chrono-gonthier | 14% | 100% | 100% | 0.98 |
| chrono-gonthier-thesis | 0% | 43% | 43% | 0.63 |
| chrono-distributed | 29% | 100% | 100% | 0.98 |
| chrono-distributed-vertex | 29% | 100% | 100% | 0.98 |
| chrono-distributed-centroid | 43% | 100% | 100% | 0.98 |
| chrono-pcm | 14% | 14% | 43% | 0.51 |
| drake-gonthier | 29% | 100% | 100% | 0.98 |
| drake-gonthier-thesis | 14% | 43% | 43% | 0.63 |
| drake-distributed | 29% | 100% | 100% | 0.98 |
| drake-distributed-vertex | 29% | 100% | 100% | 0.98 |
| drake-distributed-centroid | 29% | 100% | 100% | 0.98 |
| drake-pcm | 14% | 14% | 43% | 0.51 |
| drake-native | 14% | 14% | 43% | 0.65 |
| mujoco-gonthier | 29% | 100% | 100% | 0.98 |
| mujoco-gonthier-thesis | 14% | 43% | 43% | 0.63 |
| mujoco-distributed | 29% | 100% | 100% | 0.98 |
| mujoco-distributed-vertex | 43% | 100% | 100% | 0.98 |
| mujoco-distributed-centroid | 29% | 100% | 100% | 0.98 |
| mujoco-pcm | 14% | 14% | 43% | 0.51 |

## Convergence

Log-log slope of the error against the step (dt 2, 1, 0.5, 0.2, 0.1 ms) and against the mesh size (32, 64, 128 segments); positive means the error falls as the discretisation refines. Flags: converging, flat (|slope| ≤ 0.2 or errors at the floor), diverging, non-monotone, failed (a run is missing or not finite). Series cover only scenarios where they apply.

### By time step

| method | class | n | median slope | flags |
|---|---|---|---|---|
| chrono-distributed | A | 21 | 0.0163 | converging 6, diverging 1, flat 14 |
| chrono-distributed-centroid | A | 21 | 0.00119 | converging 3, diverging 1, flat 17 |
| chrono-distributed-vertex | A | 21 | 0.000489 | converging 3, diverging 1, flat 17 |
| chrono-gonthier | A | 21 | 0.00606 | converging 3, flat 17, non-monotone 1 |
| chrono-gonthier-thesis | A | 21 | 0.108 | converging 3, flat 16, non-monotone 2 |
| chrono-native | A | 21 | 0.00257 | converging 3, diverging 1, flat 17 |
| chrono-native-nsc | A | 21 | 5.16e-16 | converging 4, diverging 1, failed 2, flat 14 |
| chrono-pcm | A | 21 | 0.000183 | diverging 1, flat 19, non-monotone 1 |
| drake-distributed | A | 21 | 0.0262 | converging 6, diverging 1, flat 14 |
| drake-distributed-centroid | A | 21 | 0.00186 | converging 3, diverging 1, flat 17 |
| drake-distributed-vertex | A | 21 | 0.000491 | converging 3, diverging 1, flat 17 |
| drake-gonthier | A | 21 | 0.00698 | converging 3, failed 1, flat 16, non-monotone 1 |
| drake-gonthier-thesis | A | 21 | 0.0995 | converging 3, failed 1, flat 16, non-monotone 1 |
| drake-native | A | 21 | 0.15 | converging 9, diverging 1, failed 2, flat 9 |
| drake-native-point | A | 21 | 0.453 | converging 12, flat 8, non-monotone 1 |
| drake-pcm | A | 21 | 0.000183 | diverging 1, flat 19, non-monotone 1 |
| mujoco-distributed | A | 21 | 0.00219 | converging 6, diverging 1, flat 14 |
| mujoco-distributed-centroid | A | 21 | 0.00213 | converging 3, diverging 1, flat 17 |
| mujoco-distributed-vertex | A | 21 | 0.000753 | converging 3, diverging 1, flat 17 |
| mujoco-gonthier | A | 21 | 0.00606 | converging 3, flat 17, non-monotone 1 |
| mujoco-gonthier-thesis | A | 21 | 0.0956 | converging 3, failed 1, flat 16, non-monotone 1 |
| mujoco-native | A | 21 | -0.016 | diverging 2, flat 19 |
| mujoco-native-default | A | 21 | 5.16e-16 | diverging 1, flat 19, non-monotone 1 |
| mujoco-pcm | A | 21 | 0.000379 | converging 1, diverging 1, flat 18, non-monotone 1 |
| chrono-distributed | B | 5 | 5.16e-16 | flat 5 |
| chrono-distributed-centroid | B | 5 | 5.16e-16 | flat 5 |
| chrono-distributed-vertex | B | 5 | -0.00141 | flat 5 |
| chrono-gonthier | B | 5 | -0.000898 | flat 5 |
| chrono-gonthier-thesis | B | 5 | 0.00114 | flat 5 |
| chrono-native | B | 5 | -0.0529 | failed 2, flat 3 |
| chrono-native-nsc | B | 5 | -0.212 | diverging 2, failed 1, flat 2 |
| chrono-pcm | B | 5 | -0.00936 | flat 5 |
| drake-distributed | B | 5 | 5.16e-16 | flat 5 |
| drake-distributed-centroid | B | 5 | 5.16e-16 | flat 5 |
| drake-distributed-vertex | B | 5 | -0.00155 | flat 5 |
| drake-gonthier | B | 5 | -0.000898 | flat 5 |
| drake-gonthier-thesis | B | 5 | 0.00114 | flat 5 |
| drake-native | B | 5 | 0.427 | converging 3, flat 1, non-monotone 1 |
| drake-native-point | B | 5 | 0.582 | failed 3, flat 1, non-monotone 1 |
| drake-pcm | B | 5 | -0.00937 | flat 5 |
| mujoco-distributed | B | 5 | 5.16e-16 | flat 5 |
| mujoco-distributed-centroid | B | 5 | 5.16e-16 | flat 5 |
| mujoco-distributed-vertex | B | 5 | -0.00139 | flat 5 |
| mujoco-gonthier | B | 5 | -0.00159 | flat 5 |
| mujoco-gonthier-thesis | B | 5 | 0.00114 | flat 5 |
| mujoco-native | B | 5 | -0.00446 | flat 5 |
| mujoco-native-default | B | 5 | -0.0284 | flat 5 |
| mujoco-pcm | B | 5 | -0.00872 | flat 5 |
| chrono-distributed | C | 7 | 0.0244 | converging 1, flat 6 |
| chrono-distributed-centroid | C | 7 | 0.0244 | converging 1, flat 6 |
| chrono-distributed-vertex | C | 7 | 0.0244 | converging 1, flat 6 |
| chrono-gonthier | C | 7 | 0.00734 | converging 1, flat 6 |
| chrono-gonthier-thesis | C | 7 | 0.0989 | converging 3, flat 4 |
| chrono-native | C | 7 | 3.23e-07 | converging 1, failed 1, flat 5 |
| chrono-native-nsc | C | 7 | 1.09e-08 | failed 2, flat 5 |
| chrono-pcm | C | 7 | 0.321 | converging 4, diverging 1, flat 2 |
| drake-distributed | C | 7 | 0.0244 | converging 1, flat 6 |
| drake-distributed-centroid | C | 7 | 0.0244 | converging 1, flat 6 |
| drake-distributed-vertex | C | 7 | 0.0244 | converging 1, flat 6 |
| drake-gonthier | C | 7 | 0.00785 | converging 1, flat 6 |
| drake-gonthier-thesis | C | 7 | 0.0989 | converging 3, flat 4 |
| drake-native | C | 7 | -7.08e-05 | converging 2, failed 1, flat 4 |
| drake-native-point | C | 7 | 0.155 | converging 3, diverging 1, failed 1, flat 2 |
| drake-pcm | C | 7 | 0.321 | converging 4, diverging 1, flat 2 |
| mujoco-distributed | C | 7 | 0.0244 | converging 1, flat 6 |
| mujoco-distributed-centroid | C | 7 | 0.0244 | converging 1, flat 6 |
| mujoco-distributed-vertex | C | 7 | 0.0244 | converging 1, flat 6 |
| mujoco-gonthier | C | 7 | 0.00734 | converging 1, flat 6 |
| mujoco-gonthier-thesis | C | 7 | 0.0989 | converging 2, flat 4, non-monotone 1 |
| mujoco-native | C | 7 | 8.47e-05 | failed 1, flat 6 |
| mujoco-native-default | C | 7 | 4.61e-10 | failed 1, flat 6 |
| mujoco-pcm | C | 7 | 0.321 | converging 4, diverging 1, flat 2 |

### By mesh

| method | class | n | median slope | flags |
|---|---|---|---|---|
| chrono-distributed | A | 9 | 0.325 | converging 6, flat 3 |
| chrono-distributed-centroid | A | 9 | 0.325 | converging 6, flat 3 |
| chrono-distributed-vertex | A | 9 | 0.325 | converging 6, flat 3 |
| chrono-gonthier | A | 9 | 0.425 | converging 6, flat 3 |
| chrono-gonthier-thesis | A | 9 | 0.328 | converging 6, flat 3 |
| chrono-native | A | 9 | 6.49e-16 | converging 2, diverging 1, flat 6 |
| chrono-native-nsc | A | 9 | 7.7e-16 | converging 3, failed 1, flat 5 |
| chrono-pcm | A | 9 | 0.188 | converging 4, diverging 1, flat 4 |
| drake-distributed | A | 9 | 0.325 | converging 6, flat 3 |
| drake-distributed-centroid | A | 9 | 0.325 | converging 6, flat 3 |
| drake-distributed-vertex | A | 9 | 0.325 | converging 6, flat 3 |
| drake-gonthier | A | 9 | 0.425 | converging 6, flat 3 |
| drake-gonthier-thesis | A | 9 | 0.329 | converging 6, flat 3 |
| drake-native | A | 9 | 0.245 | converging 6, diverging 1, flat 2 |
| drake-native-point | A | 9 | 0.00576 | converging 3, diverging 1, flat 5 |
| drake-pcm | A | 9 | 0.188 | converging 4, diverging 1, flat 4 |
| mujoco-distributed | A | 9 | 0.325 | converging 6, flat 3 |
| mujoco-distributed-centroid | A | 9 | 0.325 | converging 6, flat 3 |
| mujoco-distributed-vertex | A | 9 | 0.325 | converging 6, flat 3 |
| mujoco-gonthier | A | 9 | 0.425 | converging 6, flat 3 |
| mujoco-gonthier-thesis | A | 9 | 0.33 | converging 6, flat 3 |
| mujoco-native | A | 9 | 7.42e-17 | diverging 4, flat 5 |
| mujoco-native-default | A | 9 | -1.19e-16 | diverging 2, flat 7 |
| mujoco-pcm | A | 9 | 0.188 | converging 4, diverging 1, flat 4 |
| chrono-distributed | B | 5 | 0.546 | converging 3, flat 1, non-monotone 1 |
| chrono-distributed-centroid | B | 5 | 0.529 | converging 2, flat 2, non-monotone 1 |
| chrono-distributed-vertex | B | 5 | 0.954 | converging 3, flat 2 |
| chrono-gonthier | B | 5 | -0.0373 | converging 1, flat 4 |
| chrono-gonthier-thesis | B | 5 | -0.00592 | converging 1, flat 4 |
| chrono-native | B | 5 | -0.803 | diverging 2, failed 2, flat 1 |
| chrono-native-nsc | B | 5 | -0.0215 | diverging 1, flat 4 |
| chrono-pcm | B | 5 | 0.869 | converging 2, flat 2, non-monotone 1 |
| drake-distributed | B | 5 | 0.548 | converging 3, flat 1, non-monotone 1 |
| drake-distributed-centroid | B | 5 | 0.531 | converging 2, flat 2, non-monotone 1 |
| drake-distributed-vertex | B | 5 | 0.941 | converging 3, flat 2 |
| drake-gonthier | B | 5 | -0.0373 | converging 1, flat 4 |
| drake-gonthier-thesis | B | 5 | -0.00592 | converging 1, flat 4 |
| drake-native | B | 5 | 0.62 | converging 5 |
| drake-native-point | B | 5 | 0.00488 | diverging 1, flat 4 |
| drake-pcm | B | 5 | 0.869 | converging 2, flat 2, non-monotone 1 |
| mujoco-distributed | B | 5 | 0.546 | converging 3, flat 1, non-monotone 1 |
| mujoco-distributed-centroid | B | 5 | 0.529 | converging 2, flat 2, non-monotone 1 |
| mujoco-distributed-vertex | B | 5 | 0.954 | converging 3, flat 2 |
| mujoco-gonthier | B | 5 | -0.0373 | converging 1, flat 4 |
| mujoco-gonthier-thesis | B | 5 | -0.00404 | converging 1, flat 4 |
| mujoco-native | B | 5 | -1.12e-16 | diverging 1, flat 4 |
| mujoco-native-default | B | 5 | -0.00553 | diverging 1, flat 4 |
| mujoco-pcm | B | 5 | 0.869 | converging 2, flat 2, non-monotone 1 |
| chrono-distributed | C | 1 | 1.84 | converging 1 |
| chrono-distributed-centroid | C | 1 | 1.84 | converging 1 |
| chrono-distributed-vertex | C | 1 | 1.84 | converging 1 |
| chrono-gonthier | C | 1 | 1.84 | converging 1 |
| chrono-gonthier-thesis | C | 1 | 1.84 | converging 1 |
| chrono-native | C | 1 | 5.28e-05 | flat 1 |
| chrono-native-nsc | C | 1 | n/a | failed 1 |
| chrono-pcm | C | 1 | -0.158 | flat 1 |
| drake-distributed | C | 1 | 1.84 | converging 1 |
| drake-distributed-centroid | C | 1 | 1.84 | converging 1 |
| drake-distributed-vertex | C | 1 | 1.84 | converging 1 |
| drake-gonthier | C | 1 | 1.84 | converging 1 |
| drake-gonthier-thesis | C | 1 | 1.84 | converging 1 |
| drake-native | C | 1 | -0.0168 | flat 1 |
| drake-native-point | C | 1 | -0.00344 | flat 1 |
| drake-pcm | C | 1 | -0.158 | flat 1 |
| mujoco-distributed | C | 1 | 1.84 | converging 1 |
| mujoco-distributed-centroid | C | 1 | 1.84 | converging 1 |
| mujoco-distributed-vertex | C | 1 | 1.84 | converging 1 |
| mujoco-gonthier | C | 1 | 1.84 | converging 1 |
| mujoco-gonthier-thesis | C | 1 | 1.84 | converging 1 |
| mujoco-native | C | 1 | -8.11e-18 | flat 1 |
| mujoco-native-default | C | 1 | 8.57e-18 | flat 1 |
| mujoco-pcm | C | 1 | -0.158 | flat 1 |

### Series to look at

| series | scenario | method | flag | slope | change last two |
|---|---|---|---|---|---|
| dt | cone | chrono-native-nsc | failed | n/a | n/a |
| dt | cone | drake-native | non-monotone | 0.735 | 0.412 |
| dt | cone | drake-native-point | failed | n/a | n/a |
| dt | contensou | chrono-native | failed | n/a | n/a |
| dt | contensou | chrono-native-nsc | diverging | -0.929 | 0.968 |
| dt | contensou | drake-native-point | failed | n/a | n/a |
| dt | disk | chrono-native | failed | n/a | n/a |
| dt | disk | chrono-native-nsc | diverging | -0.42 | 0.0617 |
| dt | disk | drake-native-point | failed | n/a | n/a |
| dt | drop | drake-native-point | diverging | -0.667 | 0.138 |
| dt | edge | mujoco-native-default | non-monotone | 0.86 | 0.971 |
| dt | energy | chrono-distributed | diverging | -0.203 | 0.567 |
| dt | energy | chrono-distributed-centroid | diverging | -0.203 | 0.567 |
| dt | energy | chrono-distributed-vertex | diverging | -0.203 | 0.567 |
| dt | energy | chrono-gonthier | non-monotone | 0.255 | 0.783 |
| dt | energy | chrono-gonthier-thesis | non-monotone | 0.255 | 0.783 |
| dt | energy | chrono-pcm | diverging | -0.216 | 0.567 |
| dt | energy | drake-distributed | diverging | -0.231 | 0.569 |
| dt | energy | drake-distributed-centroid | diverging | -0.231 | 0.569 |
| dt | energy | drake-distributed-vertex | diverging | -0.231 | 0.569 |
| dt | energy | drake-gonthier | non-monotone | 0.253 | 0.782 |
| dt | energy | drake-gonthier-thesis | non-monotone | 0.253 | 0.782 |
| dt | energy | drake-pcm | diverging | -0.215 | 0.569 |
| dt | energy | mujoco-distributed | diverging | -0.203 | 0.567 |
| dt | energy | mujoco-distributed-centroid | diverging | -0.203 | 0.567 |
| dt | energy | mujoco-distributed-vertex | diverging | -0.203 | 0.567 |
| dt | energy | mujoco-gonthier | non-monotone | 0.255 | 0.783 |
| dt | energy | mujoco-gonthier-thesis | non-monotone | 0.255 | 0.783 |
| dt | energy | mujoco-pcm | diverging | -0.216 | 0.567 |
| dt | journal | chrono-native-nsc | failed | n/a | n/a |
| dt | journal | drake-native | diverging | -0.275 | 0.0384 |
| dt | journal | mujoco-native-default | diverging | -0.541 | 0.69 |
| dt | massratio | chrono-gonthier-thesis | non-monotone | 1.69 | 1 |
| dt | massratio | chrono-pcm | non-monotone | 4.84 | 0.316 |
| dt | massratio | drake-gonthier | failed | n/a | n/a |
| dt | massratio | drake-gonthier-thesis | failed | n/a | n/a |
| dt | massratio | drake-native | failed | n/a | n/a |
| dt | massratio | drake-pcm | non-monotone | 4.84 | 0.316 |
| dt | massratio | mujoco-gonthier-thesis | failed | n/a | n/a |
| dt | massratio | mujoco-pcm | non-monotone | 4.84 | 0.316 |
| dt | oblique | chrono-native-nsc | failed | n/a | n/a |
| dt | oblique | drake-native-point | non-monotone | 0.678 | 0.211 |
| dt | push | mujoco-native | diverging | -0.601 | 0.142 |
| dt | restitution | chrono-native-nsc | failed | n/a | n/a |
| dt | restitution | chrono-pcm | diverging | -0.421 | 0.0177 |
| dt | restitution | drake-pcm | diverging | -0.421 | 0.0177 |
| dt | restitution | mujoco-pcm | diverging | -0.421 | 0.0177 |
| dt | ring | drake-native-point | non-monotone | 1.23 | 0.401 |
| dt | stack | drake-native | failed | n/a | n/a |
| dt | stack | mujoco-gonthier-thesis | non-monotone | 2.68 | 0.183 |
| dt | tip | chrono-native | failed | n/a | n/a |
| dt | tip | chrono-native-nsc | failed | n/a | n/a |
| dt | tip | drake-native-point | failed | n/a | n/a |
| dt | tip | mujoco-native | failed | n/a | n/a |
| dt | tip | mujoco-native-default | failed | n/a | n/a |
| dt | tower | drake-native | failed | n/a | n/a |
| dt | trough | chrono-native | diverging | -0.203 | 2.26e-10 |
| dt | vgroove | chrono-native-nsc | diverging | -0.885 | 0.458 |
| dt | wedge | mujoco-native | diverging | -0.39 | 0.279 |
| mesh | bearing | chrono-native | diverging | -0.803 | 0.4 |
| mesh | bearing | chrono-pcm | non-monotone | 1.13 | 0.153 |
| mesh | bearing | drake-native-point | diverging | -0.47 | 0.305 |
| mesh | bearing | drake-pcm | non-monotone | 1.1 | 0.212 |
| mesh | bearing | mujoco-pcm | non-monotone | 1.14 | 0.169 |
| mesh | billiard | drake-native | diverging | -0.534 | 0.277 |
| mesh | bowl | mujoco-native | diverging | -0.499 | 0.357 |
| mesh | cone | chrono-distributed | non-monotone | 0.546 | 0.766 |
| mesh | cone | chrono-distributed-centroid | non-monotone | 0.529 | 0.783 |
| mesh | cone | chrono-native-nsc | diverging | -0.789 | 0.604 |
| mesh | cone | drake-distributed | non-monotone | 0.548 | 0.772 |
| mesh | cone | drake-distributed-centroid | non-monotone | 0.531 | 0.788 |
| mesh | cone | mujoco-distributed | non-monotone | 0.546 | 0.767 |
| mesh | cone | mujoco-distributed-centroid | non-monotone | 0.529 | 0.783 |
| mesh | cone | mujoco-native | diverging | -2.42 | 0.235 |
| mesh | cone | mujoco-native-default | diverging | -0.561 | 0.154 |
| mesh | contensou | chrono-native | failed | n/a | n/a |
| mesh | disk | chrono-native | failed | n/a | n/a |
| mesh | energy | mujoco-native | diverging | -0.28 | 0.133 |
| mesh | journal | drake-native-point | diverging | -0.599 | 0.267 |
| mesh | journal | mujoco-native | diverging | -1.51 | 0.891 |
| mesh | journal | mujoco-native-default | diverging | -0.541 | 0.512 |
| mesh | oblique | chrono-native-nsc | failed | n/a | n/a |
| mesh | restitution | chrono-native-nsc | failed | n/a | n/a |
| mesh | ring | chrono-native | diverging | -2.11 | 0.946 |
| mesh | sphere | chrono-pcm | diverging | -0.343 | 0.133 |
| mesh | sphere | drake-pcm | diverging | -0.342 | 0.134 |
| mesh | sphere | mujoco-pcm | diverging | -0.343 | 0.133 |
| mesh | trough | chrono-native | diverging | -0.955 | 0.137 |
| mesh | trough | mujoco-native | diverging | -0.622 | 0.352 |
| mesh | trough | mujoco-native-default | diverging | -0.2 | 0.268 |

## Stiffness series

Error at dt 0.2 ms against the same step with K × 10 (class C is not run).

| scenario | method | dt 0.2 ms | K × 10 |
|---|---|---|---|
| billiard | chrono-gonthier | 0.0278 | 0.0114 |
| billiard | chrono-gonthier-thesis | 0.0278 | 0.0114 |
| billiard | chrono-distributed | 0.0105 | 0.00424 |
| billiard | chrono-distributed-vertex | 0.0105 | 0.00424 |
| billiard | chrono-distributed-centroid | 0.0105 | 0.00424 |
| billiard | chrono-pcm | 0.0172 | 0.00823 |
| billiard | chrono-native | 0.00056 | 0.000529 |
| billiard | chrono-native-nsc | 0.000649 | 0.000649 |
| billiard | drake-gonthier | 0.0278 | 0.0114 |
| billiard | drake-gonthier-thesis | 0.0278 | 0.0114 |
| billiard | drake-distributed | 0.0105 | 0.00424 |
| billiard | drake-distributed-vertex | 0.0105 | 0.00424 |
| billiard | drake-distributed-centroid | 0.0105 | 0.00424 |
| billiard | drake-pcm | 0.0172 | 0.00823 |
| billiard | drake-native | 0.0287 | 0.0138 |
| billiard | drake-native-point | 0.00112 | 0.000216 |
| billiard | mujoco-gonthier | 0.0278 | 0.0114 |
| billiard | mujoco-gonthier-thesis | 0.0278 | 0.0114 |
| billiard | mujoco-distributed | 0.0105 | 0.00424 |
| billiard | mujoco-distributed-vertex | 0.0105 | 0.00424 |
| billiard | mujoco-distributed-centroid | 0.0105 | 0.00424 |
| billiard | mujoco-pcm | 0.0172 | 0.00823 |
| billiard | mujoco-native | 0.0753 | 0.0755 |
| billiard | mujoco-native-default | 0.0845 | 0.0845 |
| block | chrono-gonthier | 0.00502 | 0.00438 |
| block | chrono-gonthier-thesis | 0.00661 | 0.00881 |
| block | chrono-distributed | 0.00477 | 0.00438 |
| block | chrono-distributed-vertex | 0.00479 | 0.00438 |
| block | chrono-distributed-centroid | 0.00479 | 0.00438 |
| block | chrono-pcm | 0.00917 | 0.00724 |
| block | chrono-native | 0.000321 | 0.000445 |
| block | chrono-native-nsc | 0.000885 | 0.000804 |
| block | drake-gonthier | 0.00503 | 0.0044 |
| block | drake-gonthier-thesis | 0.0066 | 0.00884 |
| block | drake-distributed | 0.00483 | 0.00439 |
| block | drake-distributed-vertex | 0.00481 | 0.00439 |
| block | drake-distributed-centroid | 0.00487 | 0.00446 |
| block | drake-pcm | 0.00922 | 0.00702 |
| block | drake-native | 0.00394 | 0.00222 |
| block | drake-native-point | 0.000582 | 0.000385 |
| block | mujoco-gonthier | 0.00502 | 0.00438 |
| block | mujoco-gonthier-thesis | 0.00661 | 0.00881 |
| block | mujoco-distributed | 0.0048 | 0.00438 |
| block | mujoco-distributed-vertex | 0.00482 | 0.00438 |
| block | mujoco-distributed-centroid | 0.0048 | 0.00438 |
| block | mujoco-pcm | 0.00922 | 0.00731 |
| block | mujoco-native | 0.11 | 0.118 |
| block | mujoco-native-default | 0.00495 | 0.00495 |
| bowl | chrono-gonthier | 0.153 | 0.072 |
| bowl | chrono-gonthier-thesis | 0.165 | 0.0916 |
| bowl | chrono-distributed | 0.146 | 0.0703 |
| bowl | chrono-distributed-vertex | 0.146 | 0.0703 |
| bowl | chrono-distributed-centroid | 0.146 | 0.0703 |
| bowl | chrono-pcm | 0.153 | 0.0973 |
| bowl | chrono-native | 0.0907 | 0.129 |
| bowl | chrono-native-nsc | 0.114 | 0.114 |
| bowl | drake-gonthier | 0.153 | 0.072 |
| bowl | drake-gonthier-thesis | 0.165 | 0.0916 |
| bowl | drake-distributed | 0.146 | 0.0703 |
| bowl | drake-distributed-vertex | 0.146 | 0.0703 |
| bowl | drake-distributed-centroid | 0.146 | 0.0703 |
| bowl | drake-pcm | 0.153 | 0.0973 |
| bowl | drake-native | 0.215 | 0.123 |
| bowl | drake-native-point | 0.102 | 0.108 |
| bowl | mujoco-gonthier | 0.153 | 0.072 |
| bowl | mujoco-gonthier-thesis | 0.165 | 0.0916 |
| bowl | mujoco-distributed | 0.146 | 0.0703 |
| bowl | mujoco-distributed-vertex | 0.146 | 0.0703 |
| bowl | mujoco-distributed-centroid | 0.146 | 0.0703 |
| bowl | mujoco-pcm | 0.153 | 0.0973 |
| bowl | mujoco-native | 0.401 | 0.144 |
| bowl | mujoco-native-default | 0.32 | 0.32 |
| cylinder | chrono-gonthier | 0.0023 | 0.0115 |
| cylinder | chrono-gonthier-thesis | 0.00373 | 0.0117 |
| cylinder | chrono-distributed | 0.00463 | 0.0116 |
| cylinder | chrono-distributed-vertex | 0.00463 | 0.0116 |
| cylinder | chrono-distributed-centroid | 0.00463 | 0.0116 |
| cylinder | chrono-pcm | 0.00112 | 0.00975 |
| cylinder | chrono-native | 0.00792 | 0.00762 |
| cylinder | chrono-native-nsc | 0.0226 | 0.0398 |
| cylinder | drake-gonthier | 0.0023 | 0.0115 |
| cylinder | drake-gonthier-thesis | 0.00373 | 0.0117 |
| cylinder | drake-distributed | 0.00463 | 0.0116 |
| cylinder | drake-distributed-vertex | 0.00463 | 0.0116 |
| cylinder | drake-distributed-centroid | 0.00463 | 0.0116 |
| cylinder | drake-pcm | 0.00112 | 0.00975 |
| cylinder | drake-native | 0.00869 | 0.017 |
| cylinder | drake-native-point | 0.000115 | 0.000601 |
| cylinder | mujoco-gonthier | 0.0023 | 0.0115 |
| cylinder | mujoco-gonthier-thesis | 0.00372 | 0.0117 |
| cylinder | mujoco-distributed | 0.00463 | 0.0116 |
| cylinder | mujoco-distributed-vertex | 0.00463 | 0.0116 |
| cylinder | mujoco-distributed-centroid | 0.00463 | 0.0116 |
| cylinder | mujoco-pcm | 0.00112 | 0.00975 |
| cylinder | mujoco-native | 0.000116 | 1.16e-05 |
| cylinder | mujoco-native-default | 9.64e-05 | 9.64e-05 |
| edge | chrono-gonthier | 0.188 | 0.0886 |
| edge | chrono-gonthier-thesis | 0.188 | 0.0886 |
| edge | chrono-distributed | 0.188 | 0.0886 |
| edge | chrono-distributed-vertex | 0.188 | 0.0886 |
| edge | chrono-distributed-centroid | 0.188 | 0.0886 |
| edge | chrono-pcm | 0.231 | 0.111 |
| edge | chrono-native | 0.00455 | 0.00127 |
| edge | chrono-native-nsc | 0.322 | 0.322 |
| edge | drake-gonthier | 0.188 | 0.0886 |
| edge | drake-gonthier-thesis | 0.188 | 0.0886 |
| edge | drake-distributed | 0.188 | 0.0886 |
| edge | drake-distributed-vertex | 0.188 | 0.0886 |
| edge | drake-distributed-centroid | 0.188 | 0.0886 |
| edge | drake-pcm | 0.231 | 0.111 |
| edge | drake-native | 0.265 | 0.148 |
| edge | drake-native-point | 0.00434 | 0.00132 |
| edge | mujoco-gonthier | 0.188 | 0.0886 |
| edge | mujoco-gonthier-thesis | 0.188 | 0.0886 |
| edge | mujoco-distributed | 0.188 | 0.0886 |
| edge | mujoco-distributed-vertex | 0.188 | 0.0886 |
| edge | mujoco-distributed-centroid | 0.188 | 0.0886 |
| edge | mujoco-pcm | 0.231 | 0.111 |
| edge | mujoco-native | 0.0073 | 0.00601 |
| edge | mujoco-native-default | 0.203 | 0.203 |
| energy | chrono-gonthier | 0.00612 | 0.0029 |
| energy | chrono-gonthier-thesis | 0.00612 | 0.0029 |
| energy | chrono-distributed | 0.003 | 0.00414 |
| energy | chrono-distributed-vertex | 0.003 | 0.00414 |
| energy | chrono-distributed-centroid | 0.003 | 0.00414 |
| energy | chrono-pcm | 0.0457 | 0.0346 |
| energy | chrono-native | 0.0156 | 0.00817 |
| energy | chrono-native-nsc | 0.22 | 0.22 |
| energy | drake-gonthier | 0.00615 | 0.00313 |
| energy | drake-gonthier-thesis | 0.00615 | 0.00313 |
| energy | drake-distributed | 0.003 | 0.00407 |
| energy | drake-distributed-vertex | 0.003 | 0.00407 |
| energy | drake-distributed-centroid | 0.003 | 0.00407 |
| energy | drake-pcm | 0.0458 | 0.0346 |
| energy | drake-native | 0.103 | 0.164 |
| energy | drake-native-point | 0.213 | 0.223 |
| energy | mujoco-gonthier | 0.00612 | 0.0031 |
| energy | mujoco-gonthier-thesis | 0.00612 | 0.0031 |
| energy | mujoco-distributed | 0.003 | 0.00403 |
| energy | mujoco-distributed-vertex | 0.003 | 0.00403 |
| energy | mujoco-distributed-centroid | 0.003 | 0.00403 |
| energy | mujoco-pcm | 0.0457 | 0.0346 |
| energy | mujoco-native | 0.562 | 0.276 |
| energy | mujoco-native-default | 0.471 | 0.471 |
| journal | chrono-gonthier | 0.0282 | 0.0135 |
| journal | chrono-gonthier-thesis | 0.0281 | 0.013 |
| journal | chrono-distributed | 0.0456 | 0.0109 |
| journal | chrono-distributed-vertex | 0.0456 | 0.0109 |
| journal | chrono-distributed-centroid | 0.0456 | 0.0109 |
| journal | chrono-pcm | 0.0243 | 0.0247 |
| journal | chrono-native | 0.475 | 2.57 |
| journal | chrono-native-nsc | 0.41 | 0.41 |
| journal | drake-gonthier | 0.0282 | 0.0135 |
| journal | drake-gonthier-thesis | 0.0281 | 0.013 |
| journal | drake-distributed | 0.0455 | 0.0109 |
| journal | drake-distributed-vertex | 0.0455 | 0.0109 |
| journal | drake-distributed-centroid | 0.0455 | 0.0109 |
| journal | drake-pcm | 0.0243 | 0.0246 |
| journal | drake-native | 0.0197 | 0.0391 |
| journal | drake-native-point | 0.0513 | 0.104 |
| journal | mujoco-gonthier | 0.0282 | 0.0135 |
| journal | mujoco-gonthier-thesis | 0.0281 | 0.013 |
| journal | mujoco-distributed | 0.0456 | 0.0109 |
| journal | mujoco-distributed-vertex | 0.0456 | 0.0109 |
| journal | mujoco-distributed-centroid | 0.0456 | 0.0109 |
| journal | mujoco-pcm | 0.0243 | 0.0245 |
| journal | mujoco-native | 0.0514 | 0.00252 |
| journal | mujoco-native-default | 0.0919 | 0.0919 |
| massratio | chrono-gonthier | 0.238 | 1.24e+04 |
| massratio | chrono-gonthier-thesis | 1.67e+05 | 5.98e+04 |
| massratio | chrono-distributed | 0.238 | 1.25e+04 |
| massratio | chrono-distributed-vertex | 0.238 | 1.24e+04 |
| massratio | chrono-distributed-centroid | 0.238 | 1.25e+04 |
| massratio | chrono-pcm | 0.237 | 3.31e+04 |
| massratio | chrono-native | 0.453 | 0.146 |
| massratio | chrono-native-nsc | 0.000147 | 0.000147 |
| massratio | drake-gonthier | 0.238 | 1.23e+04 |
| massratio | drake-gonthier-thesis | inf | inf |
| massratio | drake-distributed | 0.238 | 1.24e+04 |
| massratio | drake-distributed-vertex | 0.238 | 1.24e+04 |
| massratio | drake-distributed-centroid | 0.238 | 1.24e+04 |
| massratio | drake-pcm | 0.237 | 3.33e+04 |
| massratio | drake-native | 2.44e-06 | 2.58e-07 |
| massratio | drake-native-point | 78.7 | 54.5 |
| massratio | mujoco-gonthier | 0.238 | 1.24e+04 |
| massratio | mujoco-gonthier-thesis | inf | inf |
| massratio | mujoco-distributed | 0.238 | 1.25e+04 |
| massratio | mujoco-distributed-vertex | 0.238 | 1.24e+04 |
| massratio | mujoco-distributed-centroid | 0.238 | 1.25e+04 |
| massratio | mujoco-pcm | 0.237 | 3.31e+04 |
| massratio | mujoco-native | 0.000113 | 1.44e-08 |
| massratio | mujoco-native-default | 0.00716 | 0.00716 |
| oblique | chrono-gonthier | 0.0722 | 0.0322 |
| oblique | chrono-gonthier-thesis | 0.0722 | 0.0545 |
| oblique | chrono-distributed | 0.042 | 0.0186 |
| oblique | chrono-distributed-vertex | 0.042 | 0.0186 |
| oblique | chrono-distributed-centroid | 0.042 | 0.0186 |
| oblique | chrono-pcm | 0.0449 | 0.0156 |
| oblique | chrono-native | 0.000482 | 0.000537 |
| oblique | chrono-native-nsc | inf | inf |
| oblique | drake-gonthier | 0.0722 | 0.0322 |
| oblique | drake-gonthier-thesis | 0.0722 | 0.0545 |
| oblique | drake-distributed | 0.042 | 0.0186 |
| oblique | drake-distributed-vertex | 0.042 | 0.0186 |
| oblique | drake-distributed-centroid | 0.042 | 0.0186 |
| oblique | drake-pcm | 0.0449 | 0.0156 |
| oblique | drake-native | 0.0281 | 0.0136 |
| oblique | drake-native-point | 0.0127 | 0.0264 |
| oblique | mujoco-gonthier | 0.0722 | 0.0322 |
| oblique | mujoco-gonthier-thesis | 0.0722 | 0.0545 |
| oblique | mujoco-distributed | 0.042 | 0.0186 |
| oblique | mujoco-distributed-vertex | 0.042 | 0.0186 |
| oblique | mujoco-distributed-centroid | 0.042 | 0.0186 |
| oblique | mujoco-pcm | 0.0449 | 0.0156 |
| oblique | mujoco-native | 0.216 | 0.647 |
| oblique | mujoco-native-default | 0.0163 | 0.0163 |
| pieces1 | chrono-gonthier | 0.000518 | 0.000477 |
| pieces1 | chrono-gonthier-thesis | 0.000921 | 0.00808 |
| pieces1 | chrono-distributed | 0.000965 | 0.000773 |
| pieces1 | chrono-distributed-vertex | 0.000926 | 0.00112 |
| pieces1 | chrono-distributed-centroid | 0.00122 | 0.000919 |
| pieces1 | chrono-pcm | 0.0035 | 0.00218 |
| pieces1 | chrono-native | 0.00063 | 0.000786 |
| pieces1 | chrono-native-nsc | 0.00056 | 0.00056 |
| pieces1 | drake-gonthier | 0.000518 | 0.000477 |
| pieces1 | drake-gonthier-thesis | 0.000921 | 0.00808 |
| pieces1 | drake-distributed | 0.000965 | 0.000773 |
| pieces1 | drake-distributed-vertex | 0.000926 | 0.000696 |
| pieces1 | drake-distributed-centroid | 0.00122 | 0.000919 |
| pieces1 | drake-pcm | 0.0035 | 0.00218 |
| pieces1 | drake-native | 0.00814 | 0.00105 |
| pieces1 | drake-native-point | 0.00133 | 0.00116 |
| pieces1 | mujoco-gonthier | 0.000518 | 0.000477 |
| pieces1 | mujoco-gonthier-thesis | 0.000919 | 0.00808 |
| pieces1 | mujoco-distributed | 0.000964 | 0.000772 |
| pieces1 | mujoco-distributed-vertex | 0.000926 | 0.000696 |
| pieces1 | mujoco-distributed-centroid | 0.00122 | 0.000919 |
| pieces1 | mujoco-pcm | 0.0035 | 0.00218 |
| pieces1 | mujoco-native | 0.00557 | 0.00617 |
| pieces1 | mujoco-native-default | 0.343 | 0.343 |
| pieces2 | chrono-gonthier | 0.00263 | 0.00243 |
| pieces2 | chrono-gonthier-thesis | 0.0183 | 0.0316 |
| pieces2 | chrono-distributed | 0.00098 | 0.000772 |
| pieces2 | chrono-distributed-vertex | 0.0691 | 0.0571 |
| pieces2 | chrono-distributed-centroid | 0.173 | 0.138 |
| pieces2 | chrono-pcm | 0.155 | 0.15 |
| pieces2 | chrono-native | 0.0397 | 0.0501 |
| pieces2 | chrono-native-nsc | 1.08 | 1.08 |
| pieces2 | drake-gonthier | 0.00263 | 0.00243 |
| pieces2 | drake-gonthier-thesis | 0.0183 | 0.0316 |
| pieces2 | drake-distributed | 0.00098 | 0.000772 |
| pieces2 | drake-distributed-vertex | 0.0691 | 0.0573 |
| pieces2 | drake-distributed-centroid | 0.173 | 0.139 |
| pieces2 | drake-pcm | 0.155 | 0.15 |
| pieces2 | drake-native | 0.00814 | 0.00105 |
| pieces2 | drake-native-point | 0.00133 | 0.00116 |
| pieces2 | mujoco-gonthier | 0.00263 | 0.00243 |
| pieces2 | mujoco-gonthier-thesis | 0.0183 | 0.0316 |
| pieces2 | mujoco-distributed | 0.00098 | 0.000772 |
| pieces2 | mujoco-distributed-vertex | 0.0691 | 0.0578 |
| pieces2 | mujoco-distributed-centroid | 0.173 | 0.144 |
| pieces2 | mujoco-pcm | 0.155 | 0.149 |
| pieces2 | mujoco-native | 0.00557 | 0.00617 |
| pieces2 | mujoco-native-default | 0.343 | 0.343 |
| pieces64 | chrono-gonthier | 0.0702 | 0.0702 |
| pieces64 | chrono-gonthier-thesis | 0.0481 | 0.0625 |
| pieces64 | chrono-distributed | 0.000958 | 0.000741 |
| pieces64 | chrono-distributed-vertex | 0.14 | 0.122 |
| pieces64 | chrono-distributed-centroid | 0.3 | 0.257 |
| pieces64 | chrono-pcm | 0.303 | 0.297 |
| pieces64 | chrono-native | 0.183 | 0.181 |
| pieces64 | chrono-native-nsc | 1.11 | 1.11 |
| pieces64 | drake-gonthier | 0.0702 | 0.0702 |
| pieces64 | drake-gonthier-thesis | 0.0481 | 0.0625 |
| pieces64 | drake-distributed | 0.000958 | 0.000741 |
| pieces64 | drake-distributed-vertex | 0.14 | 0.122 |
| pieces64 | drake-distributed-centroid | 0.3 | 0.257 |
| pieces64 | drake-pcm | 0.303 | 0.297 |
| pieces64 | drake-native | 0.00814 | 0.00105 |
| pieces64 | drake-native-point | 0.00133 | 0.00116 |
| pieces64 | mujoco-gonthier | 0.0702 | 0.0702 |
| pieces64 | mujoco-gonthier-thesis | 0.0481 | 0.0625 |
| pieces64 | mujoco-distributed | 0.000958 | 0.000741 |
| pieces64 | mujoco-distributed-vertex | 0.14 | 0.123 |
| pieces64 | mujoco-distributed-centroid | 0.3 | 0.262 |
| pieces64 | mujoco-pcm | 0.303 | 0.296 |
| pieces64 | mujoco-native | 0.00557 | 0.00617 |
| pieces64 | mujoco-native-default | 0.343 | 0.343 |
| pieces8 | chrono-gonthier | 0.00263 | 0.00243 |
| pieces8 | chrono-gonthier-thesis | 0.0183 | 0.0316 |
| pieces8 | chrono-distributed | 0.000978 | 0.000766 |
| pieces8 | chrono-distributed-vertex | 0.0656 | 0.0585 |
| pieces8 | chrono-distributed-centroid | 0.163 | 0.136 |
| pieces8 | chrono-pcm | 0.134 | 0.13 |
| pieces8 | chrono-native | 0.0401 | 0.0494 |
| pieces8 | chrono-native-nsc | 1.09 | 1.18 |
| pieces8 | drake-gonthier | 0.00263 | 0.00243 |
| pieces8 | drake-gonthier-thesis | 0.0183 | 0.0316 |
| pieces8 | drake-distributed | 0.000978 | 0.000767 |
| pieces8 | drake-distributed-vertex | 0.0656 | 0.0583 |
| pieces8 | drake-distributed-centroid | 0.163 | 0.137 |
| pieces8 | drake-pcm | 0.134 | 0.13 |
| pieces8 | drake-native | 0.00814 | 0.00105 |
| pieces8 | drake-native-point | 0.00133 | 0.00116 |
| pieces8 | mujoco-gonthier | 0.00263 | 0.00243 |
| pieces8 | mujoco-gonthier-thesis | 0.0183 | 0.0316 |
| pieces8 | mujoco-distributed | 0.000978 | 0.000767 |
| pieces8 | mujoco-distributed-vertex | 0.0656 | 0.0585 |
| pieces8 | mujoco-distributed-centroid | 0.163 | 0.142 |
| pieces8 | mujoco-pcm | 0.134 | 0.129 |
| pieces8 | mujoco-native | 0.00557 | 0.00617 |
| pieces8 | mujoco-native-default | 0.343 | 0.343 |
| push | chrono-gonthier | 0.000306 | 0.000255 |
| push | chrono-gonthier-thesis | 0.886 | 0.889 |
| push | chrono-distributed | 0.000306 | 0.000255 |
| push | chrono-distributed-vertex | 0.000306 | 0.000255 |
| push | chrono-distributed-centroid | 0.000306 | 0.000255 |
| push | chrono-pcm | 0.958 | 0.958 |
| push | chrono-native | 0.745 | 0.23 |
| push | chrono-native-nsc | 0.000514 | 0.000514 |
| push | drake-gonthier | 0.000307 | 0.000255 |
| push | drake-gonthier-thesis | 0.886 | 0.889 |
| push | drake-distributed | 0.000307 | 0.000255 |
| push | drake-distributed-vertex | 0.000307 | 0.000255 |
| push | drake-distributed-centroid | 0.000307 | 0.000255 |
| push | drake-pcm | 0.958 | 0.958 |
| push | drake-native | 0.013 | 0.0129 |
| push | drake-native-point | 0.0129 | 0.0129 |
| push | mujoco-gonthier | 0.000306 | 0.000255 |
| push | mujoco-gonthier-thesis | 0.886 | 0.889 |
| push | mujoco-distributed | 0.000306 | 0.000255 |
| push | mujoco-distributed-vertex | 0.000306 | 0.000255 |
| push | mujoco-distributed-centroid | 0.000306 | 0.000255 |
| push | mujoco-pcm | 0.958 | 0.958 |
| push | mujoco-native | 0.0291 | 0.0314 |
| push | mujoco-native-default | 0.464 | 0.464 |
| sphere | chrono-gonthier | 0.00487 | 0.00259 |
| sphere | chrono-gonthier-thesis | 0.00776 | 0.0055 |
| sphere | chrono-distributed | 0.00612 | 0.00263 |
| sphere | chrono-distributed-vertex | 0.00612 | 0.00263 |
| sphere | chrono-distributed-centroid | 0.00612 | 0.00263 |
| sphere | chrono-pcm | 0.0167 | 0.00486 |
| sphere | chrono-native | 0.000274 | 8.28e-05 |
| sphere | chrono-native-nsc | 1.48e-06 | 6.13e-07 |
| sphere | drake-gonthier | 0.00487 | 0.0026 |
| sphere | drake-gonthier-thesis | 0.00776 | 0.0055 |
| sphere | drake-distributed | 0.00612 | 0.00263 |
| sphere | drake-distributed-vertex | 0.00612 | 0.00263 |
| sphere | drake-distributed-centroid | 0.00612 | 0.00263 |
| sphere | drake-pcm | 0.0167 | 0.00485 |
| sphere | drake-native | 0.0108 | 0.00407 |
| sphere | drake-native-point | 3.82e-05 | 5.33e-06 |
| sphere | mujoco-gonthier | 0.00487 | 0.00259 |
| sphere | mujoco-gonthier-thesis | 0.00776 | 0.0055 |
| sphere | mujoco-distributed | 0.00612 | 0.00263 |
| sphere | mujoco-distributed-vertex | 0.00612 | 0.00263 |
| sphere | mujoco-distributed-centroid | 0.00612 | 0.00263 |
| sphere | mujoco-pcm | 0.0167 | 0.00486 |
| sphere | mujoco-native | 2.2e-05 | 2.2e-06 |
| sphere | mujoco-native-default | 0.000308 | 0.000308 |
| tower | chrono-gonthier | 0.00637 | 3.21e-06 |
| tower | chrono-gonthier-thesis | 1.47e+03 | 1.14e+03 |
| tower | chrono-distributed | 0.0063 | 5.34e-05 |
| tower | chrono-distributed-vertex | 0.0063 | 5.33e-05 |
| tower | chrono-distributed-centroid | 0.0063 | 5.34e-05 |
| tower | chrono-pcm | 1.58e+03 | 1.18e+03 |
| tower | chrono-native | 10.9 | 0.576 |
| tower | chrono-native-nsc | 0.000904 | 0.000904 |
| tower | drake-gonthier | 0.00636 | 6.74e-06 |
| tower | drake-gonthier-thesis | 1.48e+03 | 1.15e+03 |
| tower | drake-distributed | 0.00631 | 5.32e-05 |
| tower | drake-distributed-vertex | 0.00631 | 5.33e-05 |
| tower | drake-distributed-centroid | 0.00631 | 5.33e-05 |
| tower | drake-pcm | 1.58e+03 | 1.18e+03 |
| tower | drake-native | inf | inf |
| tower | drake-native-point | 0.493 | 0.49 |
| tower | mujoco-gonthier | 0.00637 | 3.25e-06 |
| tower | mujoco-gonthier-thesis | 1.48e+03 | 1.16e+03 |
| tower | mujoco-distributed | 0.0063 | 5.34e-05 |
| tower | mujoco-distributed-vertex | 0.0063 | 5.33e-05 |
| tower | mujoco-distributed-centroid | 0.0063 | 5.33e-05 |
| tower | mujoco-pcm | 1.58e+03 | 1.18e+03 |
| tower | mujoco-native | 0.00783 | 0.00703 |
| tower | mujoco-native-default | 1.08 | 1.08 |
| trough | chrono-gonthier | 0.101 | 0.0596 |
| trough | chrono-gonthier-thesis | 0.183 | 0.154 |
| trough | chrono-distributed | 0.101 | 0.0597 |
| trough | chrono-distributed-vertex | 0.101 | 0.0597 |
| trough | chrono-distributed-centroid | 0.101 | 0.0597 |
| trough | chrono-pcm | 0.15 | 0.111 |
| trough | chrono-native | 0.442 | 0.823 |
| trough | chrono-native-nsc | 0.302 | 0.302 |
| trough | drake-gonthier | 0.101 | 0.0596 |
| trough | drake-gonthier-thesis | 0.183 | 0.155 |
| trough | drake-distributed | 0.101 | 0.0597 |
| trough | drake-distributed-vertex | 0.101 | 0.0597 |
| trough | drake-distributed-centroid | 0.101 | 0.0597 |
| trough | drake-pcm | 0.15 | 0.111 |
| trough | drake-native | 0.143 | 0.0994 |
| trough | drake-native-point | 0.0674 | 0.0902 |
| trough | mujoco-gonthier | 0.101 | 0.0596 |
| trough | mujoco-gonthier-thesis | 0.183 | 0.154 |
| trough | mujoco-distributed | 0.101 | 0.0597 |
| trough | mujoco-distributed-vertex | 0.101 | 0.0597 |
| trough | mujoco-distributed-centroid | 0.101 | 0.0597 |
| trough | mujoco-pcm | 0.15 | 0.111 |
| trough | mujoco-native | 0.46 | 0.135 |
| trough | mujoco-native-default | 0.249 | 0.249 |
| turntable | chrono-gonthier | 0.69 | 0.199 |
| turntable | chrono-gonthier-thesis | 0.755 | 0.253 |
| turntable | chrono-distributed | 0.69 | 0.198 |
| turntable | chrono-distributed-vertex | 0.69 | 0.198 |
| turntable | chrono-distributed-centroid | 0.69 | 0.198 |
| turntable | chrono-pcm | 0.577 | 0.16 |
| turntable | chrono-native | 1.1 | 3.4 |
| turntable | chrono-native-nsc | 0.254 | 0.254 |
| turntable | drake-gonthier | 0.69 | 0.199 |
| turntable | drake-gonthier-thesis | 0.755 | 0.253 |
| turntable | drake-distributed | 0.69 | 0.198 |
| turntable | drake-distributed-vertex | 0.69 | 0.198 |
| turntable | drake-distributed-centroid | 0.69 | 0.198 |
| turntable | drake-pcm | 0.577 | 0.16 |
| turntable | drake-native | 0.661 | 0.259 |
| turntable | drake-native-point | 0.0276 | 0.0782 |
| turntable | mujoco-gonthier | 0.69 | 0.199 |
| turntable | mujoco-gonthier-thesis | 0.755 | 0.253 |
| turntable | mujoco-distributed | 0.69 | 0.198 |
| turntable | mujoco-distributed-vertex | 0.69 | 0.198 |
| turntable | mujoco-distributed-centroid | 0.69 | 0.198 |
| turntable | mujoco-pcm | 0.577 | 0.16 |
| turntable | mujoco-native | 0.134 | 0.134 |
| turntable | mujoco-native-default | 0.149 | 0.149 |
| twoblocks | chrono-gonthier | 0.000815 | 0.000524 |
| twoblocks | chrono-gonthier-thesis | 0.0389 | 0.0327 |
| twoblocks | chrono-distributed | 0.000815 | 0.000506 |
| twoblocks | chrono-distributed-vertex | 0.000815 | 0.000506 |
| twoblocks | chrono-distributed-centroid | 0.000815 | 0.000506 |
| twoblocks | chrono-pcm | 0.0425 | 0.0411 |
| twoblocks | chrono-native | 0.0103 | 0.00503 |
| twoblocks | chrono-native-nsc | 0.000229 | 0.000229 |
| twoblocks | drake-gonthier | 0.000815 | 0.000528 |
| twoblocks | drake-gonthier-thesis | 0.0389 | 0.0327 |
| twoblocks | drake-distributed | 0.000816 | 0.00051 |
| twoblocks | drake-distributed-vertex | 0.000816 | 0.00051 |
| twoblocks | drake-distributed-centroid | 0.000816 | 0.00051 |
| twoblocks | drake-pcm | 0.0425 | 0.0411 |
| twoblocks | drake-native | 0.0072 | 0.000682 |
| twoblocks | drake-native-point | 0.000233 | 0.000233 |
| twoblocks | mujoco-gonthier | 0.000815 | 0.000524 |
| twoblocks | mujoco-gonthier-thesis | 0.0389 | 0.0327 |
| twoblocks | mujoco-distributed | 0.000815 | 0.000506 |
| twoblocks | mujoco-distributed-vertex | 0.000815 | 0.000507 |
| twoblocks | mujoco-distributed-centroid | 0.000815 | 0.000506 |
| twoblocks | mujoco-pcm | 0.0425 | 0.0411 |
| twoblocks | mujoco-native | 0.103 | 0.0845 |
| twoblocks | mujoco-native-default | 0.129 | 0.129 |
| vgroove | chrono-gonthier | 0.00138 | 0.00125 |
| vgroove | chrono-gonthier-thesis | 0.00133 | 0.0022 |
| vgroove | chrono-distributed | 0.00215 | 0.00136 |
| vgroove | chrono-distributed-vertex | 0.00223 | 0.00141 |
| vgroove | chrono-distributed-centroid | 0.00215 | 0.00135 |
| vgroove | chrono-pcm | 0.00367 | 0.00311 |
| vgroove | chrono-native | 0.00439 | 0.0036 |
| vgroove | chrono-native-nsc | 0.0445 | 0.0445 |
| vgroove | drake-gonthier | 0.00138 | 0.00125 |
| vgroove | drake-gonthier-thesis | 0.00133 | 0.00218 |
| vgroove | drake-distributed | 0.00215 | 0.00137 |
| vgroove | drake-distributed-vertex | 0.00223 | 0.00141 |
| vgroove | drake-distributed-centroid | 0.00215 | 0.00136 |
| vgroove | drake-pcm | 0.00367 | 0.00311 |
| vgroove | drake-native | 0.00676 | 0.000107 |
| vgroove | drake-native-point | 0.00761 | 0.0303 |
| vgroove | mujoco-gonthier | 0.00138 | 0.00125 |
| vgroove | mujoco-gonthier-thesis | 0.00133 | 0.0022 |
| vgroove | mujoco-distributed | 0.00215 | 0.00136 |
| vgroove | mujoco-distributed-vertex | 0.00223 | 0.00141 |
| vgroove | mujoco-distributed-centroid | 0.00215 | 0.00136 |
| vgroove | mujoco-pcm | 0.00367 | 0.00311 |
| vgroove | mujoco-native | 0.0123 | 0.0116 |
| vgroove | mujoco-native-default | 0.00141 | 0.00141 |
| wedge | chrono-gonthier | 0.000769 | 0.000486 |
| wedge | chrono-gonthier-thesis | 0.438 | 0.514 |
| wedge | chrono-distributed | 0.00516 | 0.000571 |
| wedge | chrono-distributed-vertex | 0.00515 | 0.000534 |
| wedge | chrono-distributed-centroid | 0.00517 | 0.000519 |
| wedge | chrono-pcm | 0.336 | 0.355 |
| wedge | chrono-native | 0.0442 | 0.0219 |
| wedge | chrono-native-nsc | 8.4e-05 | 7.57e-05 |
| wedge | drake-gonthier | 0.000769 | 0.000487 |
| wedge | drake-gonthier-thesis | 0.438 | 0.514 |
| wedge | drake-distributed | 0.00516 | 0.000487 |
| wedge | drake-distributed-vertex | 0.00514 | 0.000487 |
| wedge | drake-distributed-centroid | 0.00516 | 0.000487 |
| wedge | drake-pcm | 0.336 | 0.355 |
| wedge | drake-native | 0.0795 | 0.00898 |
| wedge | drake-native-point | 0.00329 | 0.00329 |
| wedge | mujoco-gonthier | 0.000769 | 0.000486 |
| wedge | mujoco-gonthier-thesis | 0.438 | 0.514 |
| wedge | mujoco-distributed | 0.00516 | 0.000486 |
| wedge | mujoco-distributed-vertex | 0.00514 | 0.000486 |
| wedge | mujoco-distributed-centroid | 0.00516 | 0.000486 |
| wedge | mujoco-pcm | 0.336 | 0.355 |
| wedge | mujoco-native | 0.0327 | 0.0371 |
| wedge | mujoco-native-default | 0.0331 | 0.0331 |
| bearing | chrono-gonthier | 0.0176 | 0.0761 |
| bearing | chrono-gonthier-thesis | 0.0169 | 0.0765 |
| bearing | chrono-distributed | 0.0159 | 0.0872 |
| bearing | chrono-distributed-vertex | 0.0159 | 0.0872 |
| bearing | chrono-distributed-centroid | 0.0159 | 0.0872 |
| bearing | chrono-pcm | 0.00203 | 0.067 |
| bearing | chrono-native | 0.209 | 0.703 |
| bearing | chrono-native-nsc | 0.989 | 0.982 |
| bearing | drake-gonthier | 0.0176 | 0.0761 |
| bearing | drake-gonthier-thesis | 0.0169 | 0.0765 |
| bearing | drake-distributed | 0.0159 | 0.0872 |
| bearing | drake-distributed-vertex | 0.0159 | 0.0872 |
| bearing | drake-distributed-centroid | 0.0159 | 0.0872 |
| bearing | drake-pcm | 0.00208 | 0.0671 |
| bearing | drake-native | 0.00975 | 0.0774 |
| bearing | drake-native-point | 0.365 | 0.0973 |
| bearing | mujoco-gonthier | 0.0175 | 0.0761 |
| bearing | mujoco-gonthier-thesis | 0.0169 | 0.0765 |
| bearing | mujoco-distributed | 0.0159 | 0.0872 |
| bearing | mujoco-distributed-vertex | 0.0159 | 0.0872 |
| bearing | mujoco-distributed-centroid | 0.0159 | 0.0872 |
| bearing | mujoco-pcm | 0.00191 | 0.0672 |
| bearing | mujoco-native | 0.868 | 0.88 |
| bearing | mujoco-native-default | 0.892 | 0.892 |
| cone | chrono-gonthier | 0.0388 | 0.357 |
| cone | chrono-gonthier-thesis | 0.0903 | 0.295 |
| cone | chrono-distributed | 0.000582 | 0.389 |
| cone | chrono-distributed-vertex | 0.00262 | 0.388 |
| cone | chrono-distributed-centroid | 0.000531 | 0.397 |
| cone | chrono-pcm | 0.0222 | 0.281 |
| cone | chrono-native | 0.0937 | 0.0355 |
| cone | chrono-native-nsc | 0.788 | 0.788 |
| cone | drake-gonthier | 0.0388 | 0.357 |
| cone | drake-gonthier-thesis | 0.0903 | 0.295 |
| cone | drake-distributed | 0.000583 | 0.389 |
| cone | drake-distributed-vertex | 0.00262 | 0.388 |
| cone | drake-distributed-centroid | 0.000531 | 0.397 |
| cone | drake-pcm | 0.0222 | 0.282 |
| cone | drake-native | 0.00604 | 0.451 |
| cone | drake-native-point | inf | inf |
| cone | mujoco-gonthier | 0.0388 | 0.36 |
| cone | mujoco-gonthier-thesis | 0.0903 | 0.299 |
| cone | mujoco-distributed | 0.000582 | 0.391 |
| cone | mujoco-distributed-vertex | 0.00262 | 0.391 |
| cone | mujoco-distributed-centroid | 0.00053 | 0.4 |
| cone | mujoco-pcm | 0.0222 | 0.282 |
| cone | mujoco-native | 0.136 | 0.749 |
| cone | mujoco-native-default | 0.354 | 0.354 |
| contensou | chrono-gonthier | 0.0888 | 0.0878 |
| contensou | chrono-gonthier-thesis | 0.139 | 0.0298 |
| contensou | chrono-distributed | 0.000578 | 0.000754 |
| contensou | chrono-distributed-vertex | 0.127 | 0.0833 |
| contensou | chrono-distributed-centroid | 0.049 | 0.0497 |
| contensou | chrono-pcm | 0.0677 | 0.0681 |
| contensou | chrono-native | inf | inf |
| contensou | chrono-native-nsc | 0.899 | 0.899 |
| contensou | drake-gonthier | 0.0888 | 0.0878 |
| contensou | drake-gonthier-thesis | 0.139 | 0.0298 |
| contensou | drake-distributed | 0.000578 | 0.000754 |
| contensou | drake-distributed-vertex | 0.127 | 0.0833 |
| contensou | drake-distributed-centroid | 0.049 | 0.0497 |
| contensou | drake-pcm | 0.0677 | 0.0681 |
| contensou | drake-native | 0.00134 | 0.00235 |
| contensou | drake-native-point | inf | 0.185 |
| contensou | mujoco-gonthier | 0.0888 | 0.088 |
| contensou | mujoco-gonthier-thesis | 0.14 | 0.0282 |
| contensou | mujoco-distributed | 0.000577 | 0.000747 |
| contensou | mujoco-distributed-vertex | 0.127 | 0.0852 |
| contensou | mujoco-distributed-centroid | 0.049 | 0.052 |
| contensou | mujoco-pcm | 0.0677 | 0.0681 |
| contensou | mujoco-native | 0.0962 | 0.104 |
| contensou | mujoco-native-default | 0.0514 | 0.0514 |
| disk | chrono-gonthier | 0.0559 | 0.196 |
| disk | chrono-gonthier-thesis | 0.335 | 0.39 |
| disk | chrono-distributed | 0.00123 | 0.181 |
| disk | chrono-distributed-vertex | 0.336 | 0.336 |
| disk | chrono-distributed-centroid | 0.0998 | 0.158 |
| disk | chrono-pcm | 0.1 | 0.1 |
| disk | chrono-native | inf | 0.229 |
| disk | chrono-native-nsc | 1.71 | 1.71 |
| disk | drake-gonthier | 0.0559 | 0.196 |
| disk | drake-gonthier-thesis | 0.335 | 0.39 |
| disk | drake-distributed | 0.00123 | 0.181 |
| disk | drake-distributed-vertex | 0.336 | 0.336 |
| disk | drake-distributed-centroid | 0.0998 | 0.158 |
| disk | drake-pcm | 0.1 | 0.1 |
| disk | drake-native | 0.00168 | 0.00174 |
| disk | drake-native-point | inf | 0.385 |
| disk | mujoco-gonthier | 0.0559 | 0.192 |
| disk | mujoco-gonthier-thesis | 0.334 | 0.39 |
| disk | mujoco-distributed | 0.00123 | 0.176 |
| disk | mujoco-distributed-vertex | 0.336 | 0.336 |
| disk | mujoco-distributed-centroid | 0.0998 | 0.152 |
| disk | mujoco-pcm | 0.1 | 0.1 |
| disk | mujoco-native | 0.337 | 0.323 |
| disk | mujoco-native-default | 0.248 | 0.248 |
| ring | chrono-gonthier | 0.00873 | 0.0466 |
| ring | chrono-gonthier-thesis | 0.0302 | 0.0656 |
| ring | chrono-distributed | 0.00072 | 0.0417 |
| ring | chrono-distributed-vertex | 0.00153 | 0.0413 |
| ring | chrono-distributed-centroid | 0.000993 | 0.0415 |
| ring | chrono-pcm | 0.00119 | 0.0237 |
| ring | chrono-native | 0.0264 | 0.0183 |
| ring | chrono-native-nsc | 0.163 | 0.163 |
| ring | drake-gonthier | 0.00873 | 0.0466 |
| ring | drake-gonthier-thesis | 0.0302 | 0.0656 |
| ring | drake-distributed | 0.00072 | 0.0417 |
| ring | drake-distributed-vertex | 0.00153 | 0.0413 |
| ring | drake-distributed-centroid | 0.000993 | 0.0415 |
| ring | drake-pcm | 0.00119 | 0.0237 |
| ring | drake-native | 0.00108 | 0.00142 |
| ring | drake-native-point | 0.00348 | 0.0422 |
| ring | mujoco-gonthier | 0.00873 | 0.0427 |
| ring | mujoco-gonthier-thesis | 0.0302 | 0.0622 |
| ring | mujoco-distributed | 0.00072 | 0.0375 |
| ring | mujoco-distributed-vertex | 0.00153 | 0.0371 |
| ring | mujoco-distributed-centroid | 0.000993 | 0.0373 |
| ring | mujoco-pcm | 0.00119 | 0.0205 |
| ring | mujoco-native | 0.175 | 0.178 |
| ring | mujoco-native-default | 0.113 | 0.113 |

## Engine independence

Median of |e₁ − e₂| / max(e₁, e₂, floor) between engines running the same law: 0 means the engine does not matter, 1 means they disagree entirely.

| law | engines | median | n |
|---|---|---|---|
| distributed | chrono/drake | 3.18e-06 | 33 |
| distributed | chrono/mujoco | 2.12e-07 | 33 |
| distributed | drake/mujoco | 5.2e-05 | 33 |
| distributed-centroid | chrono/drake | 1.34e-05 | 33 |
| distributed-centroid | chrono/mujoco | 1.3e-06 | 33 |
| distributed-centroid | drake/mujoco | 0.000169 | 33 |
| distributed-vertex | chrono/drake | 2.58e-05 | 33 |
| distributed-vertex | chrono/mujoco | 1.33e-06 | 33 |
| distributed-vertex | drake/mujoco | 0.000144 | 33 |
| gonthier | chrono/drake | 7.16e-07 | 33 |
| gonthier | chrono/mujoco | 9.39e-09 | 33 |
| gonthier | drake/mujoco | 1.3e-05 | 33 |
| gonthier-thesis | chrono/drake | 3.38e-06 | 32 |
| gonthier-thesis | chrono/mujoco | 3.36e-06 | 32 |
| gonthier-thesis | drake/mujoco | 0.00021 | 32 |
| pcm | chrono/drake | 1.1e-06 | 33 |
| pcm | chrono/mujoco | 4.05e-08 | 33 |
| pcm | drake/mujoco | 1.09e-05 | 33 |

## Perturbations

Median over scenarios of (max − min) / median error across the eight seeded draws.

| method | relative spread |
|---|---|
| chrono-distributed | 0.0433 |
| chrono-distributed-centroid | 0.0101 |
| chrono-distributed-vertex | 0.00847 |
| chrono-gonthier | 0.015 |
| chrono-gonthier-thesis | 0.0285 |
| chrono-native | 0.133 |
| chrono-native-nsc | 0.106 |
| chrono-pcm | 0.0259 |
| drake-distributed | 0.0408 |
| drake-distributed-centroid | 0.0101 |
| drake-distributed-vertex | 0.00851 |
| drake-gonthier | 0.0152 |
| drake-gonthier-thesis | 0.0278 |
| drake-native | 0.0162 |
| drake-native-point | 0.0973 |
| drake-pcm | 0.0259 |
| mujoco-distributed | 0.0433 |
| mujoco-distributed-centroid | 0.0102 |
| mujoco-distributed-vertex | 0.00847 |
| mujoco-gonthier | 0.015 |
| mujoco-gonthier-thesis | 0.0271 |
| mujoco-native | 0.0173 |
| mujoco-native-default | 0.0293 |
| mujoco-pcm | 0.0167 |

