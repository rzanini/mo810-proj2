import matplotlib.pyplot as plt

class Plotter:

    def __init__(self):
        self.pointsX = []
        self.pointsY = []

    def PlotPosition(self, robotPosition):
        self.pointsX.append(robotPosition[0])
        self.pointsY.append(robotPosition[1])

    def CreateFile(self):
        # Create a trace
        plt.plot(
            x = self.pointsX,
            y = self.pointsY,
            mode = 'markers'
        )

        plt.plotfile('Robot Position')
