#!/usr/bin/python

import matplotlib.pyplot as plt
import matplotlib.patches as patches

xylophone_keys = [
	{
		'note': 'A',
		'x': 5,
		'y': 5
	},{
		'note': 'B',
		'x': 10,
		'y': 5
	},{
		'note': 'C',
		'x': 10,
		'y': 10
	},{
		'note': 'D',
		'x': 15,
		'y': 5
	},{
		'note': 'E',
		'x': 15,
		'y': 10
	},{
		'note': 'F',
		'x': 20,
		'y': 5
	},{
		'note': 'G',
		'x': 20,
		'y': 10
	}
]

# calculate min / max of keys dataset
max_x = max(xylophone_keys,key=(lambda key: key['x']))['x']
max_y = max(xylophone_keys,key=(lambda key: key['y']))['y']

min_x = min(xylophone_keys,key=(lambda key: key['x']))['x']
min_y = min(xylophone_keys,key=(lambda key: key['y']))['y']

# some rough approximations on key width / height
key_width = (max_x - min_x) / float(len(xylophone_keys))
key_height = (max_y - min_y) / 1.5

# plot the origin
plt.scatter(0,0,marker='o')
plt.annotate('origin',(0,0),xytext=(3,3),textcoords = 'offset points')

# plot the key centers
plt.scatter(
	map(lambda key: key['x'], xylophone_keys), 
	map(lambda key: key['y'], xylophone_keys),
	marker='o')

for key in xylophone_keys:
	# label this note
	plt.annotate(key['note'],(key['x'],key['y']),xytext=(3,3),textcoords = 'offset points')

	# attempt to draw the keys
	plt.gca().add_patch(
	    patches.Rectangle(
	        (key['x'] - key_width / 2.0,key['y'] - key_height / 2.0),
	       	key_width,
	        key_height,
	        fill=False
	    )
	)

plt.show()