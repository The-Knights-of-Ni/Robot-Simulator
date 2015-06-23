# Robot-Simulator
A simulator for the FIRST Tech Challenge robotics competition

##File structure
    Robot-Simulator
        [git readme and whatnot]
        code
            [code files]
                eg.
                [platform]_main.cpp //platform layers
                simulation.h //main simulation code
                meth.h //math utilities, renamed to avoid confussion with the standard math.h
            build.bat
            build.sh
        libraries
            [any 3rd party libraries used]
        resources
            [resource files]
        test_files
            [test models and programs]
        osx_build
            [OS X build]
        win32_build
            [Windows build]
        release
            win32
                [version number]
                    [appname]-[version number]-installer.exe
                    [appname]-[version number].zip
                        [appname].exe
                        resources
                            [resources]
                        [dlls, if necessary]
            osx
                [version-number]
                    [appname].dmg
                    [appname].zip
                        [appname].app
                            Contents
                                MacOS
                                    [appname]
                                Frameworks
                                    [frameworks(osx dlls), if necessary]
                                Resources
                                    [resources]
                                Info.plist
