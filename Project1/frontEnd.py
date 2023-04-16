import backEnd
import platform
from tkinter import *
import tkinter as tk
import tkinter.ttk as ttk
from tkinter.constants import *
from threading import *


class mainWindow:
    def __init__(self, top=None):
        '''This class configures and populates the toplevel window.
           top is the toplevel containing window.'''

        top.geometry("1112x540")
        top.resizable(False, False)
        top.title("Search Algorithms for Route Navigation")
        top.configure(background="#d9d9d9")

        self.top = top
        self.combobox = tk.StringVar()
        self.style = ttk.Style()

        self.Label1 = tk.Label(self.top)
        self.Label1.configure(
            anchor='w', background="#d9d9d9", text="Choose An Algorithm", font=("", 12))
        # relx and rely Horizontal and vertical offset as a float between 0.0 and 1.0, as a fraction of the height and width of the parent widget.
        self.Label1.place(relx=0.589, rely=0.056, height=20, width=155)

        algorithmSelection = tk.StringVar()
        self.TCombobox1 = ttk.Combobox(
            self.top, width=27, textvariable=algorithmSelection)
        self.TCombobox1.place(relx=0.745, rely=0.056,
                              relheight=0.039, relwidth=0.221)
        self.TCombobox1.configure(textvariable=self.combobox, state="readonly")

        # Adding combobox drop down list
        self.TCombobox1['values'] = (
            'Uniform Cost', 'BFS', 'A* (Walking with Areal as heuristic)', 'A* (Driving with Walking as heuristic)')
        self.TCombobox1.current(0)

        self.Label2 = tk.Label(self.top)
        self.Label2.configure(anchor='w', background="#d9d9d9",
                              text="See The Results Below", font=("", 14))
        self.Label2.place(relx=0.012, rely=0.150, height=23, width=400)

        self.TLabel1 = ttk.Label(self.top)
        self.TLabel1.configure(anchor='w', background="#d9d9d9",
                               text="Welcome To Our Program!", font=("", 16))
        self.TLabel1.place(relx=0.012, rely=0.040, height=35, width=440)

        self.RUN_BUTTON = ttk.Button(self.top)
        self.style.configure("TButton", font=("", 10))
        self.RUN_BUTTON.place(relx=0.865, rely=0.111, height=45, width=86)
        self.RUN_BUTTON.configure(text="RUN!", command=self.runSimulation)

        self.style.configure('Treeview', font=("Arial", 11))
        self.memContents = ScrolledTreeView(self.top)
        self.memContents.place(relx=0.012, rely=0.207,
                               relheight=0.778, relwidth=0.975)

        # build_treeview_support starting.
        self.memContents['columns'] = (
            "Col1", "Col2", "Col3")
        self.memContents.heading("#0", text="", anchor="w")
        self.memContents.column("#0", width=0, stretch=NO)

        self.style.configure("Treeview.Heading", font=(None, 10))
        self.memContents.heading("Col1", text="Algorithm", anchor="w")
        self.memContents.heading("Col2", text="Cost", anchor="w")
        self.memContents.heading("Col3", text="Path", anchor="w")

        self.memContents.column("Col1", width="70",
                                anchor="w", minwidth="50")
        self.memContents.column("Col2", width="10",
                                anchor="w", minwidth="50")
        self.memContents.column("Col3", width="800",
                                anchor="w", minwidth="50")

################################## The simulation ##################################
    def main_test(self, source, destination):

        algotxt = self.TCombobox1.get()

        if algotxt == "BFS":
            backEnd.shared.pathList.clear()
            backEnd.shared.FilePath = "driving.txt"
            backEnd.shared.pathList = backEnd.breadthFirstSearch(
                backEnd.graphDictionary,  source,  destination)
            print(backEnd.shared.pathList)
        if algotxt == "Uniform Cost":
            backEnd.shared.pathList.clear()
            backEnd.shared.FilePath = "driving.txt"
            backEnd.uniformCost(backEnd.readingTheFile(),
                                source,  destination)
            print(backEnd.shared.pathList)
        if algotxt == "A* (Walking with Areal as heuristic)":
            backEnd.shared.pathList.clear()
            backEnd.shared.FilePath = "arealWithIndexes.txt"
            backEnd.tree = backEnd.convertMatrixToWeightedDictionary()
            backEnd.shared.FilePath = "areal.txt"
            backEnd.heuristic = backEnd.readHeuristic(destination)
            backEnd.shared.FilePath = "walking.txt"
            backEnd.shared.pathList = backEnd.AStarSearch()
            print(backEnd.shared.pathList)
        if algotxt == "A* (Driving with Walking as heuristic)":
            backEnd.shared.pathList.clear()
            backEnd.shared.FilePath = "heuristicWalkingWithIndexes.txt"
            backEnd.tree = backEnd.convertMatrixToWeightedDictionary()
            backEnd.shared.FilePath = "heuristicWalking.txt"
            backEnd.heuristic = backEnd.readHeuristic(destination)
            backEnd.shared.FilePath = "driving.txt"
            backEnd.shared.pathList = backEnd.AStarSearch()
            print(backEnd.shared.pathList)

        x = backEnd.calculateTheCost(backEnd.shared.pathList)
        print(x)
        p = backEnd.printThePath(backEnd.shared.pathList)
        print(p)
        self.memContents.insert(parent='', index='end',
                                text='', values=(algotxt, x, p))
        root.update()
        backEnd.clearPathList()
####################################################################################

    def runSimulation(self):
        info = Toplevel()
        info.geometry("437x192")
        info.configure(background="#d9d9d9")
        info.resizable(False, False)
        info.title("Pop menu")
        self.info = info
        self.combobox1 = tk.StringVar()
        self.combobox2 = tk.StringVar()
        self.style = ttk.Style()

        self.sourceInput = ttk.Combobox(self.info)
        self.sourceInput.place(relx=0.114, rely=0.469,
                               relheight=0.109, relwidth=0.327)
        self.sourceInput.configure(
            textvariable=self.combobox1, cursor="based_arrow_down", state="readonly")
        self.sourceInput['values'] = ('Aka', 'Bethlehem', 'Dura', 'Haifa', 'Halhoul', 'Hebron', 'Jenin', 'Jericho', 'Jerusalem',
                                      'Nablus', 'Nazareth', 'Qalqilya', 'Ramallah', 'Ramleh', 'Sabastia', 'Safad', 'Salfit', 'Tubas', 'Tulkarm', 'Yafa')
        self.sourceInput.current(0)

        self.goalInput = ttk.Combobox(self.info)
        self.goalInput.place(relx=0.549, rely=0.469,
                             relheight=0.109, relwidth=0.327)
        self.goalInput.configure(
            textvariable=self.combobox2, cursor="X_cursor", state="readonly")
        self.goalInput['values'] = ('Aka', 'Bethlehem', 'Dura', 'Haifa', 'Halhoul', 'Hebron', 'Jenin', 'Jericho', 'Jerusalem',
                                    'Nablus', 'Nazareth', 'Qalqilya', 'Ramallah', 'Ramleh', 'Sabastia', 'Safad', 'Salfit', 'Tubas', 'Tulkarm', 'Yafa')
        self.goalInput.current(0)

        self.Label1 = tk.Label(self.info)
        self.Label1.place(relx=0.114, rely=0.625, height=31, width=144)
        self.Label1.configure(
            anchor='center', background="#d9d9d9", text="Select The Source", font=("", 11))

        self.Label2 = tk.Label(self.info)
        self.Label2.place(relx=0.549, rely=0.625, height=31, width=147)
        self.Label2.configure(
            anchor='center', background="#d9d9d9", text="Select The Destination", font=("", 11))

        self.Label3 = tk.Label(self.info)
        self.Label3.place(relx=0.114, rely=0.156, height=51, width=334)
        self.Label3.configure(
            anchor='center', background="#d9d9d9", text="Please Enter The Source And The Destination", font=("", 11))

        def getInfo():
            backEnd.shared.source = self.sourceInput.current()
            backEnd.shared.destination = self.goalInput.current()
            info.destroy()
            self.main_test(backEnd.shared.source, backEnd.shared.destination)
        self.start = ttk.Button(self.info)
        self.style.configure("TButton", font=(
            "", 10), text="Start", cursor="bogosity")
        self.start.place(relx=0.420, rely=0.833, height=24, width=57)
        self.start.configure(text="Start", command=getInfo)
        root.update()


class AutoScroll(object):
    '''Configure the scrollbars for a widget.'''

    def __init__(self, master):

        try:
            vsb = ttk.Scrollbar(master, orient='vertical', command=self.yview)
        except:
            pass
        hsb = ttk.Scrollbar(master, orient='horizontal', command=self.xview)
        try:
            self.configure(yscrollcommand=self._autoscroll(vsb))
        except:
            pass
        self.configure(xscrollcommand=self._autoscroll(hsb))
        self.grid(column=0, row=0, sticky='nsew')
        try:
            vsb.grid(column=1, row=0, sticky='ns')
        except:
            pass
        hsb.grid(column=0, row=1, sticky='ew')
        master.grid_columnconfigure(0, weight=1)
        master.grid_rowconfigure(0, weight=1)
        # Copy geometry methods of master  (taken from ScrolledText.py)
        methods = tk.Pack.__dict__.keys() | tk.Grid.__dict__.keys() | tk.Place.__dict__.keys()
        for meth in methods:
            if meth[0] != '_' and meth not in ('config', 'configure'):
                setattr(self, meth, getattr(master, meth))

    @staticmethod
    def _autoscroll(sbar):
        '''Hide and show scrollbar as needed.'''

        def wrapped(first, last):
            first, last = float(first), float(last)
            if first <= 0 and last >= 1:
                sbar.grid_remove()
            else:
                sbar.grid()
            sbar.set(first, last)

        return wrapped

    def __str__(self):
        return str(self.master)


def _create_container(func):
    '''Creates a ttk Frame with a given master, and use this new frame to
    place the scrollbars and the widget.'''

    def wrapped(cls, master, **kw):
        container = ttk.Frame(master)
        container.bind('<Enter>', lambda e: _bound_to_mousewheel(e, container))
        container.bind(
            '<Leave>', lambda e: _unbound_to_mousewheel(e, container))
        return func(cls, container, **kw)

    return wrapped


class ScrolledTreeView(AutoScroll, ttk.Treeview):
    '''A standard ttk Treeview widget with scrollbars that will
    automatically show/hide as needed.'''

    @_create_container
    def __init__(self, master, **kw):
        ttk.Treeview.__init__(self, master, **kw)
        AutoScroll.__init__(self, master)


def _bound_to_mousewheel(event, widget):
    child = widget.winfo_children()[0]
    if platform.system() == 'Windows' or platform.system() == 'Darwin':
        child.bind_all('<MouseWheel>', lambda e: _on_mousewheel(e, child))
        child.bind_all('<Shift-MouseWheel>',
                       lambda e: _on_shiftmouse(e, child))
    else:
        child.bind_all('<Button-4>', lambda e: _on_mousewheel(e, child))
        child.bind_all('<Button-5>', lambda e: _on_mousewheel(e, child))
        child.bind_all('<Shift-Button-4>', lambda e: _on_shiftmouse(e, child))
        child.bind_all('<Shift-Button-5>', lambda e: _on_shiftmouse(e, child))


def _unbound_to_mousewheel(event, widget):
    if platform.system() == 'Windows' or platform.system() == 'Darwin':
        widget.unbind_all('<MouseWheel>')
        widget.unbind_all('<Shift-MouseWheel>')
    else:
        widget.unbind_all('<Button-4>')
        widget.unbind_all('<Button-5>')
        widget.unbind_all('<Shift-Button-4>')
        widget.unbind_all('<Shift-Button-5>')


def _on_mousewheel(event, widget):
    if platform.system() == 'Windows':
        widget.yview_scroll(-1 * int(event.delta / 120), 'units')
    elif platform.system() == 'Darwin':
        widget.yview_scroll(-1 * int(event.delta), 'units')
    else:
        if event.num == 4:
            widget.yview_scroll(-1, 'units')
        elif event.num == 5:
            widget.yview_scroll(1, 'units')


def _on_shiftmouse(event, widget):
    if platform.system() == 'Windows':
        widget.xview_scroll(-1 * int(event.delta / 120), 'units')
    elif platform.system() == 'Darwin':
        widget.xview_scroll(-1 * int(event.delta), 'units')
    else:
        if event.num == 4:
            widget.xview_scroll(-1, 'units')
        elif event.num == 5:
            widget.xview_scroll(1, 'units')


'''Main entry point for the application.'''
global root
root = tk.Tk()
root.protocol('WM_DELETE_WINDOW', root.destroy)
# Creates a toplevel widget.
global _top1, _w1
_top1 = root
_w1 = mainWindow(_top1)
root.mainloop()
