import modern_robotics as mr
import numpy as np
import numpy.linalg as lin
import matplotlib.pyplot as plt
import matplotlib.animation as animation


def printArray(arr, prec=3):
    print(np.array2string(arr, precision=prec, suppress_small=True))

def plotRefFrame(T, ax = None, frameName=None, scale = 0.1, **kwargs):
    if ax is None:
        ax = plt.gca

    R, p = mr.TransToRp(T)

    R = scale*R

    xb = R[:, 0]
    yb = R[:, 1]
    zb = R[:, 2]

    xs = xb + p
    ys = yb + p
    zs = zb + p

    ax = plt.gca()
    xh = ax.plot(np.array([p[0], xs[0]]), np.array([p[1], xs[1]]), np.array([p[2], xs[2]]), 'r', **kwargs)
    yh = ax.plot(np.array([p[0], ys[0]]), np.array([p[1], ys[1]]), np.array([p[2], ys[2]]), 'g', **kwargs)
    zh = ax.plot(np.array([p[0], zs[0]]), np.array([p[1], zs[1]]), np.array([p[2], zs[2]]), 'b', **kwargs)

    artist = [xh, yh, zh]

    if frameName is not None:
        xlabel = r"$\hat{x}_{%s}$" % frameName
        ylabel = r"$\hat{y}_{%s}$" % frameName
        zlabel = r"$\hat{z}_{%s}$" % frameName

        offset = 1.5
        x = p + offset * xb
        y = p + offset * yb
        z = p + offset * zb
        xt = ax.text(x[0], x[1], x[2], xlabel, fontsize=12, color='r')
        yt = ax.text(y[0], y[1], y[2], ylabel, fontsize=12, color='g')
        zt = ax.text(z[0], z[1], z[2], zlabel, fontsize=12, color='b')
        artist.append([xt, yt, zt])

    return artist




def plotInFrame(V, T=np.identity(4), ax = None, **kwargs):
    '''
    Plots N 3D vectors, accepts V with shapes (3,) or (3,N)
    If N=1, you can use a singleton vector.

    kwargs are the optional, variable, keyword-pair arguments you use in plotting.
    '''
    if ax is None:
       ax = plt.gca()

    if len(V.shape)==1:                     # if the vector is a singleton
        V = V.reshape((3,-1))

    # V = np.hstack((np.zeros((3,1)), V))
    V = np.vstack((V, np.ones(V.shape[1])))

    TV = T@V
    lineHandle, = ax.plot(TV[0,:],TV[1,:],TV[2,:], **kwargs)

    return lineHandle,

class Link:
    def __init__(self, screw = np.array([0,0,1,0,0,0]), linkLines = np.array([0,0,0]), theta = 0, M = np.identity(4),\
                 G = np.identity(6), Mcom = np.identity(4),\
                 color = [0,0,0],  name = None, frameName = None):
        self.name = name
        self.frameName = frameName

        self.screw = screw      # screw in link frame
        self.M = M              # config of link j in space frame in zero configuration, origin at joint
        self.T = M              # transform in current configuration
        self.theta = theta      # joint angle

        self.color = color
        self.linkLines = linkLines          # link vectors in link frame (connect the dots of each row, should include [0,0,0] if you want link drawn out from joint)
        self.__lines = None                   # plot handle for the lines

        # # Dynamic properties of the link
        self.Mcom = Mcom        # location of COM of link in joint frame. Config of COM in space frame would be T@Mcom
        self.G = G              # 6x6 block diagonal matrix Moment of Inertia and Mass of [Ib, mI]

    @property
    def screw(self):
        return self.__screw

    @screw.setter
    def screw(self, S):
        LENCHECK = (len(S)==6)
        VALIDCHECK = lin.norm(S[0:3])==1 or (lin.norm(S[0:3])==0 and lin.norm(S[3:])==1) or lin.norm(S)==0
        if LENCHECK and VALIDCHECK:
            self.__screw=S
        elif not LENCHECK:
            print('Warning: A screw must be a 6-vector')
        elif not VALIDCHECK:
            print('Warning: A screw must either have |w|=1 or |w|=0 and |v|=1')
        else:
            print('Warning: Unexpected argument for screw. Try again.')

    @property
    def theta(self):
        return self.__theta

    @theta.setter
    def theta(self, newTheta):
        if np.abs(newTheta) > 2*np.pi:
            print('Warning: Theta is greater than 2 pi. Check that angles are represented in radians.')

        self.__theta = newTheta

    @property
    def P(self):
        return self.__P

    @P.setter
    def P(self, newP):
        if mr.TestIfSE3(newP):
            self.__P = newP
        else:
            print('Warning: newM is not a homogeneous transformation matrix, SE(3).')

    def plot(self, ax = None, **kwargs):
        if ax is None:
            ax = plt.gca

        self.__lines, = plotInFrame(self.linkLines, self.T, ax=ax, color = self.color, **kwargs)

        return self.__lines


    def updatePlot(self, newT=None):
        if newT is not None:
            self.T = newT
        if self.__lines is None:
            self.__lines = plotInFrame(self.linkLines, self.T, ax=ax, color=self.color, **kwargs)

        else:
            R,p = mr.TransToRp(self.T)
            L = R@self.linkLines + p.reshape(3,-1)
            self.__lines.set_data_3d(L[0,:], L[1,:], L[2,:])

        return self.__lines


    def clear(self):
        if self.__lines is not None:
            self.__lines.remove()
            self.__lines = None


class SerialRobot():
    def __init__(self, linkList = [], Ms = np.identity(4), Mb = None):
        self.linkList = linkList
        self.Mb = Mb    # end effector configuration in zero config

        self.Ms = Ms    # space frame (in case you want to move the whole robot)
        self.Tb = np.copy(Mb)    # end effector in current configuration

        self.theta = np.zeros(len(linkList))
        self.dtheta = np.zeros(len(linkList))
        self.ddtheta = np.zeros(len(linkList))

        self.Vb = np.zeros(6) # end effector twist

        self.calcScrews()
        self.MList = self.getFrames()
        self.TList = self.MList.copy()

    def calcScrews(self):
        SList = []
        BList = []

        invMb = mr.TransInv(self.Mb)
        for link in self.linkList:
            S = mr.Adjoint(link.M)@link.screw
            B = mr.Adjoint(invMb)@S

            SList.append(S)
            BList.append(B)

        self.SList = np.stack(SList, 1)
        self.BList = np.stack(BList, 1)



    def getFrames(self):
        TList = []

        for link in self.linkList:
            TList.append(link.T)

        return TList


    def pose(self, theta = None):
    # Calculates and saves the configuration of each link
        if theta is not None:
            self.theta = theta

        newT = self.Ms
        for k, (th, S, M) in enumerate(zip(self.theta, self.SList.T, self.MList)):
            se3mat = mr.VecTose3(S * th)
            newT = newT @ mr.MatrixExp6(se3mat)
            self.TList[k] = self.linkList[k].T = newT @ M

        self.Tb = newT @ self.Mb

        return self.Tb

    def FKin(self, theta):
    # Returns the end-effector configuration for given joint angles
        return mr.FKinSpace(self.Mb, self.SList, theta)

    def plot(self, ax=None, jointFrameOn = False, bodyFrameOn = True, spaceFrameOn = True, **kwargs):
        if ax is None:
            fig, ax = plt.subplots(1, 1, figsize=(7.5, 7.5), subplot_kw={'projection': '3d'})
            ax.view_init(azim=45)
            plt.sca(ax)

            ax.set_xlim3d(-1, 1)
            ax.set_ylim3d(-1, 1)
            ax.set_zlim3d(0, 1.5)

            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_zlabel('z')

        for k, link in enumerate(self.linkList):
            link.plot(ax = ax, linewidth = 5)

            if jointFrameOn:
                plotRefFrame(link.T, link.name)

        if bodyFrameOn:
            plotRefFrame(self.Tb, 'b')

        if spaceFrameOn:
            plotRefFrame(self.Ms, 's')

        self.fig = fig = plt.gcf()
        self.ax = plt.gca()

        return fig, ax

    def anim_update(self, i):
        self.pose(self.thetaLog[:,i])
        lines = []      # holds a list of artists
        for link in self.linkList:
            lines.append(link.updatePlot())  # add the artist for the link

        for artist in self.__TbRef:
            artist.remove()

        R, p = mr.TransToRp(self.Tb)
        xb = np.stack((p, 0.1 * R[:,0] + p),1)
        yb = np.stack((p, 0.1 * R[:,1] + p),1)
        zb = np.stack((p, 0.1 * R[:,2] + p),1)

        xb_ref[0].set_data_3d(xb[0,:], xb[1,:], xb[2,:])
        yb_ref[0].set_data_3d(yb[0,:], yb[1,:], yb[2,:])
        zb_ref[0].set_data_3d(zb[0,:], zb[1,:], zb[2,:])

        return lines, xb_ref, yb_ref, zb_ref

    def anim_init(self):
        lines = []  # holds a list of artists

        for link in self.linkList:
            lines.append(link.updatePlot())  # add the artist for the link

        R,p = mr.TransToRp(self.Tb)

        xb = np.stack((p, 0.1*R[:,0] + p),1)
        yb = np.stack((p, 0.1*R[:,1] + p),1)
        zb = np.stack((p, 0.1*R[:,2] + p),1)

        xb_ref = self.ax.plot(xb[0,:], xb[1,:], xb[2,:], 'r')
        yb_ref = self.ax.plot(yb[0,:], yb[1,:], yb[2,:], 'g')
        zb_ref = self.ax.plot(zb[0,:], zb[1,:], zb[2,:], 'b')

        return lines, xb_ref, yb_ref, zb_ref

    # TODO Additional argument function for added plots (e.g. tracers, objects, etc)
    def animate(self, thetaLog=None, fps=40, ax = None, **kwargs):
        dt = int(1000/fps)

        if thetaLog is None:
            self.thetaLog = self.theta
        else:
            self.thetaLog = thetaLog

        self.pose(self.thetaLog[:,0])
        self.plot(bodyFrameOn=False, ax=ax)

        self.ani = animation.FuncAnimation(fig = self.fig, \
                                           func = self.anim_update, init_func = self.anim_init, \
                                           interval = dt, blit=False, **kwargs)
