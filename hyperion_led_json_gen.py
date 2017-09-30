import json as j

d = {
      'leds': []
  }
led_cnt = 0
SKIP=1
X = 32*SKIP
Y = 8*SKIP
for x in range(0, X, SKIP):
  if x % 2*SKIP == 0:
    yy = range (0, Y ,SKIP)
  else:
    yy = range (Y-1, -1, -SKIP)

  for y in yy:
      #print "led cnt: %d add: %d x %d" % (led_cnt, x , y)
      x1, x2 = (1.0/X) * x, (1.0/X) * (x+1)
      y1, y2 = (1.0/Y) * y, (1.0/Y) * (y+1)
      #print "rel Hor start: %f Hor end: %s" % (x1, x2)
      #print "rel Hor start: %f Hor end: %f" % (y1, y2)

      l = {
            'index': led_cnt,
            'hscan': {'minimum': x1,'maximum': x2},
            'vscan': {'minimum': y1,'maximum': y2},
        }
      d['leds'].append(l)
      led_cnt += 1
print j.dumps(d, indent=4, separators=(',', ': '))
