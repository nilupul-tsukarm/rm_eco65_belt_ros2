{"counter":158,
 "timeStamp":47981676.606300007,
 "ids":[6,7],
 "centers":[{"x":0.4284570813179016,"y":0.4792858958244324},{"x":0.23067288100719453,"y":0.7683824896812439}],
 "sizes":[{"x":0.14591510593891145,"y":0.08513960987329483},{"x":0.21769554913043977,"y":0.1535380333662033}],
 "labels":[2,3]}

'''test data from TCP：
 
{"counter":158,"timeStamp":47981676.606300007,"ids":[6,7],"centers":[{"x":0.4284570813179016,"y":0.4792858958244324},{"x":0.23067288100719453,"y":0.7683824896812439}],"sizes":[{"x":0.14591510593891145,"y":0.08513960987329483},{"x":0.21769554913043977,"y":0.1535380333662033}],"labels":[2,3]}
 
counter：JSON packet Number
 
timeStamp: not sure
 
ids: LEGO block ID (ID is assigned to each LEGO block when it is recognized)
centers: Center point information (the upper left corner of the camera image is the origin, and the lower right corner is (1,1))
sizes: bouding box of LEGO block (relative value when the whole camera image is (1,1))
 
labels:  2×2:0,  3×2:1,  4×2:2, 6×2:3 '''

{"counter":2004,
 "timeStamp":57979079.838,
 "ids":[77],
 "centers":[{"x":0.5130996108055115,"y":0.9322576522827148}],
 "sizes":[{"x":0.08471336215734482,"y":0.13572950661182404}],
 "labels":[2]}