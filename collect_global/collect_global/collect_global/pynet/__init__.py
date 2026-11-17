"""
pynet 包 - 支持各种网络通信
"""

# 版本信息
__version__ = "1.0.0"
__author__ = "LHL"

# 定义包级辅助函数
def help_info():
    """显示包的使用信息"""
    return {
        "version": __version__,
        "modules": ["redisMgr", "socket_define", "socket_rw"],
        "author": __author__
    }
