import threading
import os
import time
from ..common.logManager import output, outputMode,enum_Error
from .databaseBase import *

try:
    # MySQL数据库
    import pymysql
    from pymysql import err
except ModuleNotFoundError:
    print('install pymysql...')
    os.system('pip install pymysql')

#第二种连接mysql数据库,比pymysql快，但是需要c++环境，所以不推荐
#import mysql.connector
#from mysql.connector import Error
#from mysql.connector.pooling import MySQLConnectionPool
#############

class mysqlMgr(databaseBase):
    def __init__(self,serverName,index,db_host,db_port,db_user,db_password,db_name,databaseMgr):
        super().__init__(serverName,index,"mysql",db_host,db_port,db_user,db_password,db_name,database_type.mysql,databaseMgr)
    #检查数据库连接
    def check_connection(connect):
        bRes = False
        if connect is not None:
            try:
                with connect.cursor() as cursor:
                    # 执行查询
                    cursor.execute("select 1")
                    connect.commit()
                    bRes = True
            except Exception as e:
                output(f"[异常]:[mysqlMgr.check_connection]:{e}",outputMode.saveLog,enum_Error.localErr)
        return bRes

    #得到数据库连接,用于执行操作,属于一次执行
    def getConnect(db_host,db_port,db_user,db_password,db_name):
        connection = None
        try:
            connection = pymysql.connect(host=db_host,port= db_port,
                                        user=db_user,password=db_password,
                                        database=db_name,charset='utf8mb4',
                                        cursorclass=pymysql.cursors.DictCursor)
        except Exception as e:
            output(f"[异常]:[mysqlMgr.getConnect]:[{db_host,db_port,db_name}]:{e}",outputMode.saveLog,enum_Error.localErr)
        return connection
    def getClass(self):
        return mysqlMgr

    def getTable_field(connection,tableName):
        res={}
        try:
            with connection.cursor() as cursor:
                query = """
                SELECT 
                    COLUMN_NAME,
                    COLUMN_TYPE,
                    DATA_TYPE,
                    IS_NULLABLE,
                    COLUMN_DEFAULT,
                    COLUMN_KEY,
                    EXTRA,
                    COLUMN_COMMENT
                FROM INFORMATION_SCHEMA.COLUMNS
                WHERE TABLE_SCHEMA = %s
                    AND TABLE_NAME = %s
                ORDER BY ORDINAL_POSITION
                """
                cursor.execute(query, (database, table_name))
                columns = cursor.fetchall()
    
                for col in columns:
                    res[col[0]]={
                        'full_type'     : col[1],
                        'type'          : col[2],
                        'not_null'      : bool(col[3]),
                        'default_value' : col[4],
                        'is_primary_key': bool(col[5]),
                        'extra'         : col[6],
                        'comment'       : col[7]
                    }
                #cursor.execute(f"DESCRIBE {table_name}")
                #columns = cursor.fetchall()

                #for col in columns:
                #    res[col[0]] ={
                #        'name': col[1],
                #        'type': col[2],
                #        'not_null': bool(col[3]),
                #        'default_value': col[4],
                #        'is_primary_key': bool(col[5])
                #    }
        except Exception as e:
            output(f"[异常]:[sqliteMgr.getTable_field]:{e}",outputMode.saveLog,enum_Error.localErr)
        return res
    def getConnectInterface(self):
        return mysqlMgr.getConnect(self.db_host,self.db_port,self.db_user,self.db_password,self.db_name)
    def closeConnect(connection):
        try:
            if connection is not None:  connection.close()
            connection = None
        except Exception as e:
            output(f"[异常]:[mysqlMgr.closeConnect]:{e}",outputMode.saveLog,enum_Error.localErr)
        return None
    def clearTable(connection,tableName):
        try:
            with connection.cursor() as cursor:
                # 执行查询
                cursor.execute(f"truncate table {tableName}")
                connection.commit()
        except Exception as e:
            output(f"[异常]:[mysqlMgr.clearTable]:[清空表{tableName}时失败]:{e}",outputMode.saveLog,enum_Error.localErr)
    def copyTable(connection,tableName,new_tableName):
        bRes = False
        try:
            with connection.cursor() as cursor:
                # 执行查询
                cursor.execute(f"create table IF NOT EXISTS {new_tableName} like {tableName}")
                connection.commit()
                bRes = True
        except Exception as e:
            output(f"[异常]:[mysqlMgr.copyTable]:[复制表{tableName}时失败]:{e}",outputMode.saveLog,enum_Error.localErr)
        return bRes
    def executeSql(connection,sql,bCommit = True,bGetRes = True):
        results = []
        try:
            with connection.cursor() as cursor:
                if cursor is not None:
                    # 执行查询
                    cursor.execute(sql)
                    # 获取所有结果
                    if bGetRes: results = cursor.fetchall()
                    if bCommit: connection.commit()
        except err.ProgrammingError as e:
            output(f"[异常]:[mysqlMgr.executeSql.ProgrammingError]:{e}\n{sql}",outputMode.saveLog,enum_Error.localErr)
        except Exception as e:
            output(f"[异常]:[mysqlMgr.executeSql.Exception]:{e}\n{sql}",outputMode.saveLog,enum_Error.localErr)
        return results

    def singleQuery(db_host,db_port,db_user,db_password,db_db,sql,bCommit = True,bGetRes = True):
        results = None
        try:
            with mysqlMgr.getConnect(db_host,db_port,db_user,db_password,db_db) as connection:
                return mysqlMgr.executeSql(connection,sql,bCommit,bGetRes)
        except Exception as e:
            output(f"[异常]:[mysqlMgr.singleQuery.Exception]:{e}\n{sql}",outputMode.saveLog,enum_Error.localErr)
        return results

    def dualThread(self):
        db_host     = self.db_host
        db_port     = self.db_port
        db_user     = self.db_user
        db_password = self.db_password
        db_name     = self.db_name
        """处理接收"""
        output(f"[mysqlMgr.dualThread]:[{db_host,db_port,db_name}]启动",outputMode.saveLog,enum_Error.localInfo)
        conn = None
        while self.is_run:
            try:
                # 连接数据库
                #conn = pymysql.connect(
                #    host=db_host,                           # 主机名
                #    user=db_user,                           # 用户名
                #    password=db_password,                   # 密码
                #    database=db_name,                       # 数据库名（可选）
                #    port=db_port,                           # 端口（默认3306）
                #    charset='utf8mb4',                      # 字符集
                #    cursorclass=pymysql.cursors.DictCursor  # 返回字典格式的结果
                #)
                conn = self.getConnectInterface()
                if conn is None:
                    time.sleep(1)
                    continue
                conn.autocommit = False
                nWriteTimeout = self.write_Event_timeout_ms / 1000 #转换成秒

                nTotalWriteCount = 1000
                nCurWriteCount = 0
                while self.is_run:
                    nCurWriteCount = nCurWriteCount+1
                    if nCurWriteCount> nTotalWriteCount:
                        break
                    self.writeEvent.wait(nWriteTimeout)
                    self.writeEvent.clear()
                    if self.is_run is False:    break
                    # 执行普通批量更新慢
                    if self.databaseMgr is None:    continue
                    #复制插入记录
                    writeList = self.databaseMgr.copyWriteData()
                    while True:
                        index =None
                        if not writeList:   break#没有数据退出循环
                        try:
                            curT = tools.timeManager.get_timestamp_ms() 
                            #output(f'[开始时间{tools.timeManager.format_time_ms(curT/1000)}][mysql 数据长度]:{len(self.writeList)} -处理长度{len(writeList)}',outputMode.saveLog,enum_Error.localInfo.value) 
                            
                            with conn.cursor() as cursor:
                                for index, write in enumerate(writeList, 1):
                                #for write in writeList:
                                    cursor.execute(write)
                                    bCommit = True
                                    # 每 nGroupNum 条语句提交一次
                                    if index % self.nGroupNum == 0:
                                        index = None
                                        bCommit = False
                                        conn.commit()
                            
                            curEndT = tools.timeManager.get_timestamp_ms() 
                            #output(f'[普通语句][开始时间:{tools.timeManager.format_time_ms(curT/1000)}]-[结束时间:{tools.timeManager.format_time_ms(curEndT/1000)}]-[耗时:{curEndT-curT}]-[执行记录数:{len(writeList)}]',outputMode.saveLog,enum_Error.localInfo) 
                            #output(f'[普通语句][耗时:{curEndT-curT}]-[执行记录数:{len(writeList)}]',outputMode.saveLog,enum_Error.localInfo.value) 

                            # 提交剩余的语句
                            index = None
                            writeList.clear()
                            if bCommit is True: conn.commit()
                            break
                        except Exception as e:
                            # 发生异常时回滚
                            conn.rollback()
                            tools.output(f"[异常]:[mysqlMgr.dualThread]:[执行SQL批量写入时发生错误]:{e}",outputMode.saveLog,enum_Error.localErr)
                            if mysqlMgr.check_connection(conn) == False:
                                with self.writeLock:
                                    for tmp in writeList:   self.databaseMgr.writeSql(tmp)
                                break
                            
                            if index is not None:
                                try:
                                    del writeList[index-1]#删除错误语句，从新执行插入
                                except Exception as e:
                                    output(f"[异常]:[mysqlMgr.dualThread]:[执行SQL批量写入时发生错误]:{e}\n{writeList[index-1]}",outputMode.saveLog,enum_Error.localErr)
                            else:
                                break
                conn = mysqlMgr.closeConnect(conn)
            except Exception as e:
                output(f"[异常]:[mysqlMgr.dualThread]:[{self._name}]:[执行整体操作错误]:{e}",outputMode.saveLog,enum_Error.localErr)
        conn = mysqlMgr.closeConnect(conn)
        output(f"[mysqlMgr.dualThread]:[{db_host,db_port,db_name}]线程停止",outputMode.saveLog,enum_Error.localInfo)