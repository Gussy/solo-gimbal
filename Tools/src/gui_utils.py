from PySide import QtCore, QtGui

def getComboboxSelection(combobox):
    return combobox.itemText(combobox.currentIndex())

def waitCursor(function):
    def wrapper(*args, **kwargs):
        # QtGui.QApplication.setOverrideCursor(QtGui.QCursor(QtCore.Qt.WaitCursor))
        # # Update the cursor position to the restoreOverrideCursor() method update on screen
        # x, y = QtGui.QCursor.pos().x(), QtGui.QCursor.pos().y()
        # QtGui.QCursor.setPos(x, y)
        result = function(*args, **kwargs)
        # QtGui.QApplication.restoreOverrideCursor()
        # # Update the cursor position to the restoreOverrideCursor() method update on screen
        # x, y = QtGui.QCursor.pos().x(), QtGui.QCursor.pos().y()
        # QtGui.QCursor.setPos(x, y)
        return result
    return wrapper
