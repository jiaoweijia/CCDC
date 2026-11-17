import sqlite3

import re
import os
from typing import Tuple, List, Optional
from ..common import commonTools as tools
from ..pynet.redisMgr import *
from .databaseBase import *

# 获取 SQLite 库的版本号（字符串形式）
#output(f"SQLite 版本 (字符串):{sqlite3.sqlite_version}", outputMode.saveLog,enum_Error.localInfo)
class sqliteMgr(databaseBase):
    def __init__(self, serverName,index,db_host,db_port,db_user,db_password,db_name,databaseMgr):
        super().__init__(serverName,index,"sqlite",db_host,db_port,db_user,db_password,db_name,database_type.sqlite,databaseMgr)
    def check_connection(connect):
        bRes = False
        if connect is not None:
            try:
                cursor = connect.cursor()
                if cursor:
                    # 执行查询
                    cursor.execute("select 1")
                    connect.commit()
                    bRes = True
            except Exception as e:
                output(f"[sqliteMgr.check_connection]:[错误]:{e}",outputMode.saveLog,enum_Error.localErr)
        return bRes
    def getClass(self):
        return sqliteMgr

    def getTable_field(connection,table_name):
        res={}
        try:
            cursor = connection.cursor()
            cursor.execute(f"PRAGMA table_info({table_name})")
            columns = cursor.fetchall()

            for col in columns:
                res[col[1]] ={
                    'id': col[0],
                    'type': col[2],
                    'not_null': bool(col[3]),
                    'default_value': col[4],
                    'is_primary_key': bool(col[5])
                }
        except Exception as e:
            output(f"[sqliteMgr.getTable_field]:[错误]:{e}",outputMode.saveLog,enum_Error.localErr)
        return res

    def copyTable(connection,tableName,new_tableName):
        """
        在SQLite中创建一个与原表结构相同的新表
        :param conn: SQLite连接对象
        :param original_table: 原表名
        :param new_table: 新表名
        """
        cursor = connection.cursor()
    
        try:
            cursor.execute(f"SELECT type,name,sql FROM sqlite_master WHERE tbl_name='{tableName}'")
            connection.commit()
            result = cursor.fetchall()
            if not result:
                raise ValueError(f"原表 {tableName} 不存在")

            for row in result:
                tag = row[0]
                strName = row[1]
                create_sql = row[2]
                if create_sql is None or tag is None or strName is None:
                    continue;
                new_create_sql = ''
                
                if tag == 'table':
                    if create_sql.startswith("CREATE TABLE"):
                        # 替换原表名为新表名
                        new_create_sql = create_sql.replace(
                            f'CREATE TABLE "{strName}"', 
                            f'CREATE TABLE IF NOT EXISTS "{new_tableName}"'
                        )
                    else:
                        output(f"[sqliteMgr.copyTable]获取 {tableName} 的创建语句失败",outputMode.saveLog,enum_Error.localInfo)
                elif tag == 'index':
                    if create_sql.startswith("CREATE INDEX"):
                        # 替换原表名为新表名
                        new_create_sql = create_sql.replace(
                            f'CREATE INDEX "{strName}"', 
                            f'CREATE INDEX IF NOT EXISTS "{strName}_{new_tableName}"'
                        )
                        new_create_sql = new_create_sql.replace(
                            f'ON "{tableName}"', 
                            f'ON "{new_tableName}"'
                        )
                        
                    else:
                        output(f"获取 {tableName}{row[0]} 的创建语句失败",outputMode.saveLog,enum_Error.localInfo)
                # 执行创建新表的SQL
                if not tools.is_empty(new_create_sql):
                    cursor.execute(new_create_sql)
                    connection.commit()
                output(f"表 {new_tableName}{row[0]} 已成功创建，结构与 {tableName} 一致",outputMode.saveLog,enum_Error.localInfo)

            ## 1. 获取原表的创建语句
            #cursor.execute(f"SELECT sql FROM sqlite_master WHERE type='table' AND name='{tableName}'")
            #result = cursor.fetchone()
        
            #if not result:
            #    raise ValueError(f"原表 {tableName} 不存在")
        
            #create_sql = result[0]
        
            ## 2. 修改SQL语句中的表名为新表名，并添加IF NOT EXISTS
            ## 处理CREATE TABLE后面的表名
            #if create_sql.startswith("CREATE TABLE"):
            #    # 替换原表名为新表名
            #    new_create_sql = create_sql.replace(
            #        f'CREATE TABLE "{tableName}"', 
            #        f'CREATE TABLE IF NOT EXISTS "{new_tableName}"'
            #    )
            #    # 执行创建新表的SQL
            #    cursor.execute(new_create_sql)
            #    connection.commit()
            #    print(f"表 {new_tableName} 已成功创建，结构与 {tableName} 一致")
            #else:
            #    raise ValueError(f"获取 {tableName} 的创建语句失败")
            
        except Exception as e:
            connection.rollback()
            print(f"[sqliteMgr]:创建表失败: {str(e)}")
        finally:
            cursor.close()
        #try:
        #    cursor = connection.cursor()
        #    # 执行查询
        #    cursor.execute(f"CREATE TABLE IF NOT EXISTS {new_tableName} AS SELECT * FROM {tableName} WHERE 1=0;")
        #    connection.commit()
        #except Exception as e:
        #    output(f"复制表{tableName}时失败,错误:{e}",outputMode.saveLog,enum_Error.localInfo)
        #pass
    def getConnectInterface(self):
        return sqliteMgr.getConnect(self.db_host,self.db_port,self.db_user,self.db_password,self.db_name)
    def clearTable(connection,tableName):
        try:
            with connection.cursor() as cursor:
                # 开始事务
                cursor.execute("BEGIN TRANSACTION;")
                uuidTable = tools.getUUIDString()
                # 执行所有步骤
                cursor.execute(f"CREATE TABLE IF NOT EXISTS {uuidTable} AS SELECT * FROM {table_name} WHERE 1=0;")
                cursor.execute(f"DROP TABLE {table_name};")
                cursor.execute(f"CREATE TABLE {table_name} AS SELECT * FROM {uuidTable};")
                cursor.execute("DROP TABLE {uuidTable};")
        
                # 提交事务
                connection.commit()
        except Exception as e:
            output(f"[sqliteMgr]:清空表{tableName}时失败,错误:{e}",outputMode.saveLog,enum_Error.localInfo)
    @staticmethod
    def getConnect(db_host,db_port,db_user,db_password,db_name):
        connection = None
        try:
            connection = sqlite3.connect(db_name)
            connection.row_factory = sqlite3.Row  # 使查询结果支持按字段名访问
            connection.execute("PRAGMA journal_mode=WAL;")  # 只需执行一次
        except Exception as e:
            output(f"[异常]:[sqliteMgr.getConnect]:{e}",outputMode.saveLog,enum_Error.localErr)
        return connection
    @staticmethod
    def closeConnect(connection):
        try:
            if connection  is not None:  connection.close()
            connection = None
        except Exception as e:
            output(f"[异常]:[sqliteMgr.closeConnect]:{e}",outputMode.saveLog,enum_Error.localErr)
        return None
    @staticmethod
    def closeCursor(cursor):
        try:
            if cursor is not None:  cursor.close()
            cursor = None
        except Exception as e:
            output(f"[异常]:[sqliteMgr.closeCursor]:{e}",outputMode.saveLog,enum_Error.localErr)
        return None
    @staticmethod
    def executeSql(connection,sql,bCommit = True,bGetRes = True):
        res = []
        cursor = None
        try:
            cursor = connection.cursor()
            if cursor:
                cursor.execute(sql)
                if bGetRes: res = sqliteMgr.dict_fetchall(cursor)#没有设置row_factory 需要手动转换，推荐connection.row_factory = sqlite3.Row用法
                connection.commit()
        except Exception as e:
            output(f"[sqliteMgr]:sqlite查询sql错误:{sql}\n错误:{e}",outputMode.saveLog,enum_Error.localInfo)
        cursor = sqliteMgr.closeCursor(cursor)  #手动关闭游标
        return res
    @staticmethod
    def updateRecord(conn,cursor,sql)->Tuple[bool, str]:
        bRes = False
        strErr =''
        try:
            cursor.execute(sql)
            conn.commit()
        except Exception as e:
            res = []
            print(e)
        return bRes,strErr
    
    @staticmethod
    def initDbByFile(db_path,sql_file_path,reset=False) -> Tuple[bool, str]:
        """
        处理并执行Navicat导出的SQL文件
        :param sql_file_path: SQL文件路径
        :param db_path: 目标SQLite数据库路径（默认与SQL文件同目录）
        """
        bRes = False
        strErr =''

        if reset is True:
            dir_path = os.path.dirname(db_path)
            tools.fileManager.removeDir(dir_path)
            tools.fileManager.createDir(dir_path)

        # 确定数据库路径
        if not db_path:
            db_dir  = os.path.dirname(sql_file_path)
            db_name = os.path.splitext(os.path.basename(sql_file_path))[0] + ".db"
            db_path = os.path.join(db_dir, db_name)
    
        # 读取SQL文件内容
        try:
            with open(sql_file_path, 'r', encoding='utf-8') as f:
                sql_content = f.read()
        except UnicodeDecodeError:
            # 尝试其他编码（如GBK）
            with open(sql_file_path, 'r', encoding='gbk') as f:
                sql_content = f.read()
        except Exception as e:
            strErr =f"读取SQL文件失败: {e}"
            return bRes,strErr
    
        # 1. 预处理：移除注释
        # 移除/*...*/块注释（包括多行）
        sql_content = re.sub(r'/\*.*?\*/', '', sql_content, flags=re.DOTALL)
        # 移除--行注释
        sql_content = re.sub(r'--.*$', '', sql_content, flags=re.MULTILINE)
    
        # 2. 预处理：SQLite兼容性调整
        # 替换AUTO_INCREMENT为AUTOINCREMENT（SQLite关键字）
        sql_content = sql_content.replace('AUTO_INCREMENT', 'AUTOINCREMENT')
        # 移除MySQL特有语法（如ENGINE、DEFAULT CHARSET）
        sql_content = re.sub(r'ENGINE=.*?(;| )', '\\1', sql_content)
        sql_content = re.sub(r'DEFAULT CHARSET=.*?(;| )', '\\1', sql_content)
    
        # 3. 分割SQL语句（按;分割，忽略字符串中的;）
        # 正则说明：匹配不在引号内的;
        sql_statements = re.split(r';(?=(?:[^"\']*["\'][^"\']*["\'])*[^"\']*$)', sql_content)
    
        # 4. 执行SQL语句
        conn = None
        try:
            conn = sqlite3.connect(db_path)
            cursor = conn.cursor()
        
            success_count = 0
            for stmt in sql_statements:
                stmt = stmt.strip()
                if not stmt:    continue
                # 执行语句
                cursor.execute(stmt)
                success_count += 1
                display_stmt = stmt[:100] + "..." if len(stmt) > 100 else stmt
            conn.commit()
            bRes = True
        except sqlite3.Error as e:
            if conn:    conn.rollback()
            strErr = f"执行失败: {e}"
        finally:
            conn = sqliteMgr.closeConnect(conn)
        return bRes,strErr
    
    def dict_fetchall(cursor):
        """将游标查询结果转换为字典列表"""
        if not cursor.description:  return []
        columns = [col[0].lower() for col in cursor.description]  # 获取列名
        return [dict(zip(columns, row)) for row in cursor.fetchall()]

    def dualThread(self):
        db_host     = self.db_host
        db_port     = self.db_port
        db_user     = self.db_user
        db_password = self.db_password
        db_name     = self.db_name
        """处理接收"""
        output(f"[sqliteMgr.dualThread]:{db_host,db_port,db_name}成功启动",outputMode.saveLog,enum_Error.localInfo)
        conn = None
        while self.is_run:
            try:
                conn = sqliteMgr.getConnect(db_host,db_port,db_user,db_password,db_name)
                nWriteTimeout = self.write_Event_timeout_ms / 1000 #转换成秒
                nTotalWriteCount = 1000
                nCurWriteCount = 0
                while self.is_run:
                    nCurWriteCount = nCurWriteCount+1
                    if nCurWriteCount> nTotalWriteCount:    break
                    self.writeEvent.wait(nWriteTimeout)
                    self.writeEvent.clear()
                    # 执行普通批量更新慢
                    if self.databaseMgr is None:    continue
                    writeList = self.databaseMgr.copyWriteData()
                    while True:
                        index =None
                        if not writeList:   break
                        try:
                            curT = tools.timeManager.get_timestamp_ms() 
                            #output(f'[开始时间{tools.timeManager.format_time_ms(curT/1000)}][mysql 数据长度]:{len(self.writeList)} -处理长度{len(writeList)}',outputMode.saveLog,enum_Error.localInfo.value) 
                            cursor = conn.cursor()
                            bStartTransaction = False
                            for index, write in enumerate(writeList, 1):
                                if bStartTransaction == False:
                                    conn.execute("BEGIN TRANSACTION;")
                                    bStartTransaction = True

                                cursor.execute(write)
                                bCommit = True
                                # 每 nGroupNum 条语句提交一次
                                if index % self.nGroupNum == 0:
                                    index = None
                                    bCommit = False
                                    conn.commit()
                                    bStartTransaction= False
                           
                            # 提交剩余的语句
                            if bCommit is True: conn.commit()
                                                            
                            curEndT = tools.timeManager.get_timestamp_ms() 
                            #output(f'[普通语句][开始时间:{tools.timeManager.format_time_ms(curT/1000)}]-[结束时间:{tools.timeManager.format_time_ms(curEndT/1000)}]-[耗时:{curEndT-curT}]-[执行记录数:{len(writeList)}]',outputMode.saveLog,enum_Error.localInfo) 
                            #output(f'[普通语句][耗时:{curEndT-curT}]-[执行记录数:{len(writeList)}]',outputMode.saveLog,enum_Error.localInfo.value) 
                            index = None
                            writeList.clear()
                            break
                        except sqlite3.OperationalError as e:
                            # 发生异常时回滚
                            conn.rollback()
                            if "locked" in str(e):
                                time.sleep(0.1)
                            else:
                                tools.output(f"[sqlite3.OperationalError]错误: {e}",outputMode.saveLog,enum_Error.localInfo)
                                if sqliteMgr.check_connection(conn) == False:
                                    with self.writeLock:
                                        for tmp in writeList:
                                            self.databaseMgr.writeSql(tmp)
                                    break
                            
                                if index is not None:
                                    try:
                                        del writeList[index-1]#删除错误语句，从新执行插入
                                    except Exception as e:
                                        output(f"[sqliteMgr]:执行SQL批量写入时发生错误: {e},{writeList[index-1]}",outputMode.saveLog,enum_Error.localInfo)
                                else:
                                    break
                        except Exception as e:
                            # 发生异常时回滚
                            conn.rollback()
                            tools.output(f"[sqliteMgr]:执行SQL批量写入时发生错误: {e}",outputMode.saveLog,enum_Error.localInfo)
                            if sqliteMgr.check_connection(conn) == False:
                                with self.writeLock:
                                    for tmp in writeList:   self.databaseMgr.writeSql(tmp)
                                break
                            
                            if index is not None:
                                try:
                                    del writeList[index-1]#删除错误语句，从新执行插入
                                except Exception as e:
                                    output(f"[sqliteMgr]:执行SQL批量写入时发生错误: {e},{writeList[index-1]}",outputMode.saveLog,enum_Error.localInfo)
                            else:
                                break
                conn = sqliteMgr.closeConnect(conn)
            except Exception as e:
                output(f"[sqliteMgr]:{self._name}:连接数据库时发生错误: {e}",outputMode.saveLog,enum_Error.localInfo)
        conn = sqliteMgr.closeConnect(conn)
        output(f"[sqliteMgr]:写sqlite {db_host,db_port,db_name} 停止",outputMode.saveLog,enum_Error.localInfo)