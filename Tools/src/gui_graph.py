import time
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

class GraphWindow(object):
    def __init__(self):
        self.win = pg.GraphicsWindow()
        self.win.setWindowTitle('3DR Jimbal Graphs')

        # Plot in chunks, adding one new plot curve for every 'chunkSize' samples
        self.chunkSize = 100

        # Number of seconds to display + 1
        self.maxChunks = 21

        self.startTime = pg.ptime.time()

    def __del__(self):
        self.win.close()
        self.win.hide()
        self.win.destroy()

    def newGraph(self, title):
        graph = dict()
        graph['graph'] = self.win.addPlot()
        graph['graph'].setLabel('left', title, 'radians')
        graph['graph'].setLabel('bottom', 'Time', 's')
        graph['graph'].setXRange(-self.maxChunks + 1, 0)
        graph['plots'] = dict()
        graph['data'] = np.empty((self.chunkSize + 1, 2))
        graph['pointer'] = 0
        graph['curves'] = []
        graph['min'] = 0
        graph['max'] = 0
        self.win.nextRow()
        return graph

    def updateGraph(self, graph, time, value):
        for c in graph['curves']:
            c.setPos(-(time-self.startTime), 0)
        
        i = graph['pointer'] % self.chunkSize
        if i == 0:
            curve = graph['graph'].plot()
            graph['curves'].append(curve)
            last = graph['data'][-1]
            graph['data'] = np.empty((self.chunkSize + 1,2))        
            graph['data'][0] = last
            while len(graph['curves']) > self.maxChunks:
                c = graph['curves'].pop(0)
                graph['graph'].removeItem(c)
        else:
            curve = graph['curves'][-1]
        graph['data'][i+1,0] = time - self.startTime
        graph['data'][i+1,1] = value
        curve.setData(
            x=graph['data'][:i+2, 0],
            y=graph['data'][:i+2, 1]
        )
        graph['pointer'] += 1

        # Handle background colouring
        # if value < 0:
        #     graph['graph'].getViewBox().setBackgroundColor('#ff0000')
        # elif abs(value) > 2:
        #     graph['graph'].getViewBox().setBackgroundColor('#0000ff')
        # else:
        #     graph['graph'].getViewBox().setBackgroundColor('#00ff00')

    # def updateMinMax(self, graph, value):
    #     if value < graph['min']:
    #         graph['min'] = value
    #     elif value > graph['max']:
    #         graph['max'] = value

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    gw, pitch, roll, yaw = None, None, None, None
    def updateGraphs():
        global gw, pitch, roll, yaw

        now = time.time()

        pitchValue = np.random.normal()
        gw.updateGraph(pitch, now, pitchValue)

        rollValue = np.random.normal()
        gw.updateGraph(roll, now, rollValue)

        yawValue = np.random.normal()
        gw.updateGraph(yaw, now, yawValue)

    gw = GraphWindow()
    pitch = gw.newGraph('Pitch gyro')
    roll = gw.newGraph('Roll gyro')
    yaw = gw.newGraph('Yaw gyro')

    # update all plots
    timer = pg.QtCore.QTimer()
    timer.timeout.connect(updateGraphs)
    timer.start(100)

    QtGui.QApplication.instance().exec_()
