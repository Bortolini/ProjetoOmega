#-------------------------------------------------
#
# Project created by QtCreator 2013-12-13T18:42:35
#
#-------------------------------------------------

QT       += core gui

TARGET = ProjetoOmega
TEMPLATE = app


SOURCES += main.cpp\
        projetoomega.cpp \
    LoadMap.cpp \
    myscene.cpp \
    pioneernavigation.cpp

HEADERS  += projetoomega.h \
    LoadMap.h \
    myscene.h \
    pioneernavigation.h

FORMS    += projetoomega.ui

OTHER_FILES +=

RESOURCES += \
    mapa01.qrc

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../usr/local/Aria/lib/release/ -lAria
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../usr/local/Aria/lib/debug/ -lAria
else:symbian: LIBS += -lAria
else:unix: LIBS += -L$$PWD/../../../../usr/local/Aria/lib/ -lAria

INCLUDEPATH += $$PWD/../../../../usr/local/Aria/include
DEPENDPATH += $$PWD/../../../../usr/local/Aria/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../usr/local/Aria/lib/release/ -lArNetworking
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../usr/local/Aria/lib/debug/ -lArNetworking
else:symbian: LIBS += -lArNetworking
else:unix: LIBS += -L$$PWD/../../../../usr/local/Aria/lib/ -lArNetworking

INCLUDEPATH += $$PWD/../../../../usr/local/Aria/include
DEPENDPATH += $$PWD/../../../../usr/local/Aria/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../usr/local/Aria/lib/release/ -lAria
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../usr/local/Aria/lib/debug/ -lAria
else:symbian: LIBS += -lAria
else:unix: LIBS += -L$$PWD/../../../../usr/local/Aria/lib/ -lAria

INCLUDEPATH += $$PWD/../../../../usr/local/Aria/include
DEPENDPATH += $$PWD/../../../../usr/local/Aria/include

LIBS += `pkg-config opencv --libs`
