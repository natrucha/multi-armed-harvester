import matplotlib.pyplot as plt

# important references
# https://stackoverflow.com/questions/19184484/how-to-add-group-labels-for-bar-charts-in-matplotlib
# https://chrisalbon.com/python/data_visualization/matplotlib_percentage_stacked_bar_plot/

# to save as PDF (and other options)
# https://stackoverflow.com/questions/37288421/how-to-plot-a-chart-in-the-terminal

class plotStates(object):
    def __init__(self, states_percent, file_name):
        '''
           Plot the percent time each arm spends in each of the six given
           states: idle, pick in yz, pick in x, grab, retract, drop off.
        '''
        # for plotting
        self.idle_plot   = []
        self.pickyz_plot = []
        self.pickx_plot  = []
        self.grab_plot   = []
        self.retr_plot   = []
        self.unlo_plot   = []

        self.file_name = file_name

        # arm state lists lists the % time the arm is in each state
        for arm_state_lists in states_percent:
            self.idle_plot.append(arm_state_lists[0])
            self.pickyz_plot.append(arm_state_lists[1])
            self.pickx_plot.append(arm_state_lists[2])
            self.grab_plot.append(arm_state_lists[3])
            self.retr_plot.append(arm_state_lists[4])
            self.unlo_plot.append(arm_state_lists[5])

        ## FOR MELON ONLY ##
        melon = 0
        if melon == 1:
            # reverse the lists, since the indexing is reversed
            self.idle_plot.reverse()
            self.pickyz_plot.reverse()
            self.pickx_plot.reverse()
            self.grab_plot.reverse()
            self.retr_plot.reverse()
            self.unlo_plot.reverse()

        self.plot()


    def plot(self):
        # stacked bar plot x-labels
        # add no. arms and no. of cells to manipulate the plot x-labels

        data = {'Crop row':
                   {'rear (k=11)':0,
                    'k=10':1,
                    'k=9':2,
                    'k=8':3,
                    'k=7':4,
                    'k=6':5,
                    'k=5':6,
                    'k=4':7,
                    'k=3':8,
                    'k=2':9,
                    'k=1':10,
                    'front (k=0)':11
                   }
                }

        # data = {'Bottom':
        #            {'rear':0,
        #             'mid0':1,
        #             'mid1':2,
        #             'front':3
        #            },
        # #         'Middle Bt':
        # #            {'rear':0,
        # #             'mid0':1,
        # #             'mid1':2,
        # #             'mid2':3,
        # #             'front':4
        # #            },
        # #         'Middle Top':
        # #            {'rear':0,
        # #             'mid0':1,
        # #             'mid1':2,
        # #             'mid2':3,
        # #             'front':4
        # #            },
        #         'Middle':
        #            {'rear':0,
        #             'mid0':1,
        #             'mid1':2,
        #             'front':3
        #            },
        #         'Top':
        #            {'rear':0,
        #             'mid0':1,
        #             'mid1':2,
        #             'front':3
        #            },
        #        }

        # Create a figure with a single subplot
        f, ax = plt.subplots(1, figsize=(10,5))

        # Set bar width at 1
        bar_width = 1

        # positions of the left bar-boundaries
        bar_l = [i for i in range(1,len(self.idle_plot)+1)]

        # positions of the x-axis ticks (center of the bars as bar labels)
        tick_pos = [i+(bar_width/2) for i in bar_l]
        # print(tick_pos)

        # Create the total steps per state
        totals = [i+j+k+l+m+n for i,j,k,l,m,n in zip(self.idle_plot, self.pickyz_plot, self.pickx_plot, self.grab_plot, self.retr_plot, self.unlo_plot)]

        # Create the percentage of the total steps for idle state
        per_idle = [i / j * 100 for  i,j in zip(self.idle_plot, totals)]
        # Create the percentage of the total steps for pickingyz state
        per_pickyz = [i / j * 100 for  i,j in zip(self.pickyz_plot, totals)]
        # Create the percentage of the total steps for pickingx state
        per_pickx = [i / j * 100 for  i,j in zip(self.pickx_plot, totals)]
        # Create the percentage of the total steps for grabbing state
        per_grab = [i / j * 100 for  i,j in zip(self.grab_plot, totals)]
        # Create the percentage of the total steps for retracting state
        per_retr = [i / j * 100 for  i,j in zip(self.retr_plot, totals)]
        # Create the percentage of the total steps for unloading state
        per_unlo = [i / j * 100 for  i,j in zip(self.unlo_plot, totals)]

        # Create a bar chart in position bar_1
        ax.bar(bar_l,
               # using idle data
               per_idle,
               # labeled
               label='Idle',
               # with alpha
               alpha=0.9,
               # with color
               color='#6B4C9A',
               # with bar width
               width=bar_width,
               # with border color
               edgecolor='white'
               )

        ax.bar(bar_l,
               # using picking data
               per_pickyz,
               # with per_idle
               bottom=per_idle,
               # labeled
               label='Active',
            #    label='PickingYZ',
               # with alpha
               alpha=0.9,
               # with color
               color='#396AB1',
               # with bar width
               width=bar_width,
               # with border color
               edgecolor='white'
               )

        ax.bar(bar_l,
               # using picking data
               per_pickx,
               # with per_idle
               bottom=[i+j for i,j in zip(per_idle, per_pickyz)],
               # labeled
            #    label='PickingX',
               # with alpha
               alpha=0.9,
               # with color
               color='#3E9651',
               # with bar width
               width=bar_width,
               # with border color
               edgecolor='white'
               )

        ax.bar(bar_l,
               # using picking data
               per_grab,
               # with per_idle
               bottom=[i+j+k for i,j,k in zip(per_idle, per_pickyz, per_pickx)],
               # labeled
            #    label='Grabbing',
               # with alpha
               alpha=0.9,
               # with color
               color='#948B3D',
               # with bar width
               width=bar_width,
               # with border color
               edgecolor='white'
               )

        # Create a bar chart in position bar_1
        ax.bar(bar_l,
               # using retracting data
               per_retr,
               # with per_idle and per_pick on bottom
               bottom=[i+j+k+l for i,j,k,l in zip(per_idle, per_pickyz, per_pickx, per_grab)],
               # labeled
            #    label='Retracting',
               # with alpha
               alpha=0.9,
               # with color
               color='#DA7C30',
               # with bar width
               width=bar_width,
               # with border color
               edgecolor='white'
               )

        # Create a bar chart in position bar_1
        ax.bar(bar_l,
               # using retracting data
               per_unlo,
               # with all other percents on bottom
               bottom=[i+j+k+l+m for i,j,k,l,m in zip(per_idle, per_pickyz, per_pickx, per_grab, per_retr)],
               # labeled
            #    label='Unloading',
               # with alpha
               alpha=0.9,
               # with color
               color='#CC2529',
               # with bar width
               width=bar_width,
               # with border color
               edgecolor='white'
               )


        ax.set_ylabel("Percent [%]")

        # grouping x-axis values
        self.label_group_bar(ax, data)
        f.subplots_adjust(bottom=0.3, top=0.9, right=0.8)
        # f.subplots_adjust()

        # rotate axis labels
        # plt.setp(plt.gca().get_xticklabels(), rotation=90, horizontalalignment='right')

        # create legend outside of plot
        # see https://stackoverflow.com/questions/4700614/how-to-put-the-legend-out-of-the-plot/43439132#43439132
        ax.legend(bbox_to_anchor=(1.2, 1), loc='upper right', ncol=1)

        # save the plot in high resolution
        plt.savefig(self.file_name,dpi=300)
        # shot plot
        plt.show()

    def mk_groups(self, data):
        # function takes a dictionary (or anything with an items() method, like collections.OrderedDict) and converts it to a data format that is then used to create the chart. It is basically a list
        try:
            newdata = data.items()
        except:
            return

        thisgroup = []
        groups = []
        for key, value in newdata:
            newgroups = self.mk_groups(value)
            if newgroups is None:
                thisgroup.append((key, value))
            else:
                thisgroup.append((key, len(newgroups[-1])))
                if groups:
                    groups = [g + n for n, g in zip(newgroups, groups)]
                else:
                    groups = newgroups
        return [thisgroup] + groups


    ## Functions to add grouping into the arm state percentage plot (and other future plots)
    def add_line(self, ax, xpos, ypos):
        # creates a vertical line in the subplot at the specified positions (in axes coordinates)
        line = plt.Line2D([xpos, xpos], [ypos + .1, ypos],
                          transform=ax.transAxes, color='black')
        line.set_clip_on(False)
        ax.add_line(line)


    def label_group_bar(self, ax, data):
        # takes a dictionary and creates the subplot with the labels beneath
        groups = self.mk_groups(data)
        xy = groups.pop()
        x, y = zip(*xy)
        ly = len(y)
        # xticks = range(1, ly + 2)
        xticks = range(1, ly+1) ## these values seem to change, not sure why. ##

        # print('what is y?', y)
        # print('what is ly?', ly)
        # print('giving me issues: x', x)
        # print('number of x ticks?', xticks)

        ax.set_xticks(xticks)
        ax.set_xticklabels(x)
        ax.set_xlim(.5, ly + .5)

        scale = 1. / ly
        for pos in range(ly + 1):
            self.add_line(ax, pos * scale, -.1)

        ypos = -.2
        while groups:
            group = groups.pop()
            pos = 0
            for label, rpos in group:
                lxpos = (pos + .5 * rpos) * scale
                ax.text(lxpos, ypos, label, ha='center', transform=ax.transAxes)
                self.add_line(ax, pos*scale, ypos)
                pos += rpos
            self.add_line(ax, pos*scale, ypos)
            ypos -= .1
