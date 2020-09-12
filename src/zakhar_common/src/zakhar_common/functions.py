import os, sys

def zkros_get_workspace_path():
    ZAKHAROS_WS = None
    TOP_DIR = os.path.abspath("/")
    here = os.path.dirname(os.path.realpath(__file__))
    while True:
        # print("Check:" + str(here))
        if here == TOP_DIR:
            print("Not found the workspace")
            break

        c_ws = os.path.join(here, ".catkin_workspace")
        if os.path.isfile(c_ws):
            ZAKHAROS_WS = here
            break
        else:
            here = os.path.dirname(here)
    return ZAKHAROS_WS

def zkros_import_workspace_site_packages():
    zakharos_ws = None
    TOP_DIR = os.path.abspath("/")
    here = os.path.dirname(os.path.realpath(__file__))
    while True:
        # print("Check:" + str(here))
        if here == TOP_DIR:
            print("Not found the workspace")
            break

        c_ws = os.path.join(here, ".catkin_workspace")
        if os.path.isfile(c_ws):
            zakharos_ws = here
            break
        else:
            here = os.path.dirname(here)

    if zakharos_ws:
        DEVEL_SITE_PACKAGES = os.path.join(zakharos_ws, "devel", "lib", "site-packages")
        sys.path.append(DEVEL_SITE_PACKAGES)
        return True
    else:
        return False