import threading
import os
import time
from ..common.logManager import output, outputMode,enum_Error
from .databaseBase import *
import sys
try:
    # MySQL数据库
    import pymssql
    
except ModuleNotFoundError:
    print('install pymssql...')
    #if sys.version.find('3.7.') != -1:
    # os.system('pip install -i https://pypi.tuna.tsinghua.edu.cn/simple pymssql==2.1.4')
    os.system('pip install -i https://pypi.tuna.tsinghua.edu.cn/simple pymssql==2.3.11')
    #else:
    #    os.system('pip install pymssql')

class sqlserverMgr(databaseBase):
    def __init__(self,serverName,index,db_host,db_port,db_user,db_password,db_name,databaseMgr):
        super().__init__(serverName,index,"sqlserver",db_host,db_port,db_user,db_password,db_name,database_type.sqlserver,databaseMgr)
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
                output(f"[sqlserverMgr]检查数据库连接:{e}",outputMode.saveLog,enum_Error.localInfo)
        return bRes
    #得到数据库连接,用于执行操作,属于一次执行
    def getConnect(db_host,db_port,db_user,db_password,db_name):
        connection = None
        try:
            connection = pymssql.connect(
                        host=db_host,
                        port =db_port,
                        user=db_user,
                        password=db_password,
                        database=db_name,
                    )
        except Exception as e:
            output(f"[sqlserverMgr]连接数据库:{db_host,db_port,db_name},错误:{e}",outputMode.saveLog,enum_Error.localInfo)
        return connection
    def getClass(self):
        return sqlserverMgr
    def getTable_field(connection,tableName):
        res={}
        try:
            with connection.cursor() as currsor:
                strSql = f"""
                SELECT tc.CONSTRAINT_NAME,tc.CONSTRAINT_TYPE,ccu.COLUMN_NAME
FROM INFORMATION_SCHEMA.TABLE_CONSTRAINTS tc INNER JOIN INFORMATION_SCHEMA.CONSTRAINT_COLUMN_USAGE ccu 
    ON tc.CONSTRAINT_NAME = ccu.CONSTRAINT_NAME WHERE tc.TABLE_NAME = '{table_name}' AND tc.TABLE_SCHEMA = 'dbo';"""
                cursor.execute(strSql)
                columns = cursor.fetchall()   
            #cursor.execute(f"DESCRIBE {table_name}")
            #columns = cursor.fetchall()

                for col in columns:
                    res[col[2]] ={
                        'type': col[1],
                    }
        except Exception as e:
            output(f"[sqliteMgr]:[getTable_field]:{e}",outputMode.saveLog,enum_Error.localErr)
        return res
    def getConnectInterface(self):
        return sqlserverMgr.getConnect(self.db_host,self.db_port,self.db_user,self.db_password,self.db_name)
    def closeConnect(connection):
        try:
            if connection is not None:  connection.close()
            connection = None
        except Exception as e:
            output(f"[sqlserverMgr]写sqlserver {db_host,db_port,db_name} 启动",outputMode.saveLog,enum_Error.localInfo)
        return None
    def clearTable(connection,tableName):
        try:
            with connection.cursor() as cursor:
                # 执行查询
                cursor.execute(f"truncate table {tableName}")
                connection.commit()
        except Exception as e:
            output(f"[sqlserverMgr]清空表{tableName}时失败,错误:{e}",outputMode.saveLog,enum_Error.localInfo)
    def copyTable(connection,tableName,new_tableName):
        bRes = False
        try:
            with connection.cursor() as cursor:
                # 执行查询
                # str_sql = f'''
                # IF NOT EXISTS (
                #                     SELECT * FROM INFORMATION_SCHEMA.TABLES 
                #                     WHERE TABLE_SCHEMA = 'dbo' 
                #                     AND TABLE_NAME = '{new_tableName}'
                #                )
                #     BEGIN
                #         -- 复制表结构
                #         SELECT * INTO dbo.{new_tableName} 
                #         FROM dbo.{tableName} 
                #         WHERE 1 = 0;
    
                #         -- 如果有需要，可以在这里复制索引、约束等
                #         -- 例如添加一个自增主键
                #         IF NOT EXISTS (
                #             SELECT * FROM sys.columns 
                #             WHERE object_id = OBJECT_ID('dbo.{new_tableName}') 
                #             AND is_identity = 1
                #         )
                #         BEGIN
                #             ALTER TABLE dbo.{new_tableName} 
                #             ADD ID INT IDENTITY(1,1);
        
                #             ALTER TABLE dbo.{new_tableName} 
                #             ADD CONSTRAINT PK_{new_tableName} PRIMARY KEY (ID);
                #         END
    
                #         PRINT '表 {new_tableName} 创建完成';
                #     END
                #  ELSE
                #     BEGIN
                #         PRINT '表 {new_tableName} 已存在';
                #     END
                # '''

                ###########
                # str_sql = f'''
                # IF NOT EXISTS (
                #                     SELECT * FROM INFORMATION_SCHEMA.TABLES 
                #                     WHERE TABLE_SCHEMA = 'dbo' 
                #                     AND TABLE_NAME = '{new_tableName}'
                #                )
                #     BEGIN
                #         -- 复制表结构
                #         SELECT * INTO dbo.{new_tableName} 
                #         FROM dbo.{tableName} 
                #         WHERE 1 = 0;
    
                #         -- 如果有需要，可以在这里复制索引、约束等

    
                #         PRINT '表 {new_tableName} 创建完成';
                #     END
                #  ELSE
                #     BEGIN
                #         PRINT '表 {new_tableName} 已存在';
                #     END
                # '''
                str_sql = f'''
                                        -- 声明变量
                        DECLARE @tableName NVARCHAR(128) = '{tableName}';
                        DECLARE @new_tableName NVARCHAR(128) ='{new_tableName}';
                        DECLARE @schemaName NVARCHAR(128) = 'dbo';

                        IF NOT EXISTS (
                            SELECT * FROM INFORMATION_SCHEMA.TABLES 
                            WHERE TABLE_SCHEMA = @schemaName 
                            AND TABLE_NAME = @new_tableName
                        )
                        BEGIN
                            -- 复制表结构
                            DECLARE @sql NVARCHAR(MAX);
                            SET @sql = 'SELECT * INTO ' + QUOTENAME(@schemaName) + '.' + QUOTENAME(@new_tableName) 
                                       + ' FROM ' + QUOTENAME(@schemaName) + '.' + QUOTENAME(@tableName) 
                                       + ' WHERE 1 = 0';
                            EXEC sp_executesql @sql;
    
                            PRINT '表 ' + @new_tableName + ' 结构创建完成';
    
                            -- 获取源表的所有索引
                            DECLARE @indexes TABLE (
                                index_id INT,
                                index_name NVARCHAR(255),
                                is_primary_key BIT,
                                is_unique BIT,
                                is_unique_constraint BIT,
                                type_desc NVARCHAR(60),
                                columns NVARCHAR(MAX),
                                included_columns NVARCHAR(MAX),
                                filter_definition NVARCHAR(MAX),
                                is_disabled BIT
                            );
    
                            -- 查询所有索引信息
                            INSERT INTO @indexes
                            SELECT 
                                i.index_id,
                                i.name,
                                i.is_primary_key,
                                i.is_unique,
                                i.is_unique_constraint,
                                i.type_desc,
                                -- 键列
                                STUFF((
                                    SELECT ', ' + QUOTENAME(c.name) + 
                                           CASE WHEN ic.is_descending_key = 1 THEN ' DESC' ELSE ' ASC' END
                                    FROM sys.index_columns ic
                                    JOIN sys.columns c ON ic.object_id = c.object_id AND ic.column_id = c.column_id
                                    WHERE ic.object_id = i.object_id 
                                      AND ic.index_id = i.index_id
                                      AND ic.is_included_column = 0
                                    ORDER BY ic.key_ordinal
                                    FOR XML PATH('')
                                ), 1, 2, ''),
                                -- 包含列
                                CASE 
                                    WHEN EXISTS (
                                        SELECT 1 FROM sys.index_columns ic2 
                                        WHERE ic2.object_id = i.object_id 
                                          AND ic2.index_id = i.index_id 
                                          AND ic2.is_included_column = 1
                                    ) THEN
                                        STUFF((
                                            SELECT ', ' + QUOTENAME(c.name)
                                            FROM sys.index_columns ic
                                            JOIN sys.columns c ON ic.object_id = c.object_id AND ic.column_id = c.column_id
                                            WHERE ic.object_id = i.object_id 
                                              AND ic.index_id = i.index_id
                                              AND ic.is_included_column = 1
                                            FOR XML PATH('')
                                        ), 1, 2, '')
                                    ELSE NULL
                                END,
                                i.filter_definition,
                                i.is_disabled
                            FROM sys.indexes i
                            WHERE i.object_id = OBJECT_ID(QUOTENAME(@schemaName) + '.' + QUOTENAME(@tableName))
                              AND i.type IN (1, 2)  -- 聚集索引(CLUSTERED)和非聚集索引(NONCLUSTERED)
                              AND i.is_hypothetical = 0  -- 排除虚拟索引
                            ORDER BY i.is_primary_key DESC, i.index_id;
    
                            -- 声明游标变量
                            DECLARE @index_id INT;
                            DECLARE @index_name NVARCHAR(255);
                            DECLARE @is_primary_key BIT;
                            DECLARE @is_unique BIT;
                            DECLARE @is_unique_constraint BIT;
                            DECLARE @type_desc NVARCHAR(60);
                            DECLARE @columns NVARCHAR(MAX);
                            DECLARE @included_columns NVARCHAR(MAX);
                            DECLARE @filter_definition NVARCHAR(MAX);
                            DECLARE @is_disabled BIT;
    
                            -- 创建游标遍历所有索引
                            DECLARE index_cursor CURSOR FOR 
                            SELECT * FROM @indexes;
    
                            OPEN index_cursor;
                            FETCH NEXT FROM index_cursor INTO @index_id, @index_name, @is_primary_key, @is_unique, 
                                                             @is_unique_constraint, @type_desc, @columns, 
                                                             @included_columns, @filter_definition, @is_disabled;
    
                            WHILE @@FETCH_STATUS = 0
                            BEGIN
                                -- 1. 如果是主键
                                IF @is_primary_key = 1
                                BEGIN
                                    SET @sql = 'ALTER TABLE ' + QUOTENAME(@schemaName) + '.' + QUOTENAME(@new_tableName) 
                                             + ' ADD CONSTRAINT ' + QUOTENAME(@index_name + '_'+@new_tableName+ '_PK') 
                                             + ' PRIMARY KEY ' + @type_desc 
                                             + ' (' + @columns + ')';
            
                                    EXEC sp_executesql @sql;
                                    PRINT '  主键索引创建完成: ' + @index_name;
                                END
                                -- 2. 如果是唯一约束
                                ELSE IF @is_unique_constraint = 1
                                BEGIN
                                    SET @sql = 'ALTER TABLE ' + QUOTENAME(@schemaName) + '.' + QUOTENAME(@new_tableName) 
                                             + ' ADD CONSTRAINT ' + QUOTENAME(@index_name + '_'+@new_tableName+ '_UQ') 
                                             + ' UNIQUE ' + @type_desc 
                                             + ' (' + @columns + ')';
            
                                    EXEC sp_executesql @sql;
                                    PRINT '  唯一约束索引创建完成: ' + @index_name;
                                END
                                -- 3. 如果是普通索引
                                ELSE
                                BEGIN
                                    SET @sql = 'CREATE ' 
                                             + CASE WHEN @is_unique = 1 THEN 'UNIQUE ' ELSE '' END
                                             + @type_desc + ' INDEX ' + QUOTENAME(@index_name+ '_'+@new_tableName+ '_IDX') 
                                             + ' ON ' + QUOTENAME(@schemaName) + '.' + QUOTENAME(@new_tableName) 
                                             + ' (' + @columns + ')';
            
                                    -- 添加包含列
                                    IF @included_columns IS NOT NULL
                                        SET @sql = @sql + ' INCLUDE (' + @included_columns + ')';
            
                                    -- 添加筛选条件（筛选索引）
                                    IF @filter_definition IS NOT NULL
                                        SET @sql = @sql + ' WHERE ' + @filter_definition;
            
                                    -- 如果是禁用的索引
                                    IF @is_disabled = 1
                                        SET @sql = @sql + ' WITH (ONLINE = OFF)';
            
                                    EXEC sp_executesql @sql;
                                    PRINT '  普通索引创建完成: ' + @index_name;
                                END
        
                                FETCH NEXT FROM index_cursor INTO @index_id, @index_name, @is_primary_key, @is_unique, 
                                                                 @is_unique_constraint, @type_desc, @columns, 
                                                                 @included_columns, @filter_definition, @is_disabled;
                            END
    
                            CLOSE index_cursor;
                            DEALLOCATE index_cursor;
    
                            -- 复制外键约束（可选）
                            PRINT '开始复制外键约束...';
                            DECLARE @fk_name NVARCHAR(255);
                            DECLARE @parent_columns NVARCHAR(MAX);
                            DECLARE @referenced_table NVARCHAR(255);
                            DECLARE @referenced_columns NVARCHAR(MAX);
                            DECLARE @update_rule NVARCHAR(60);
                            DECLARE @delete_rule NVARCHAR(60);
    
                            DECLARE fk_cursor CURSOR FOR 
                            SELECT 
                                fk.name,
                                STUFF((
                                    SELECT ', ' + QUOTENAME(c.name)
                                    FROM sys.foreign_key_columns fkc
                                    JOIN sys.columns c ON fkc.parent_object_id = c.object_id AND fkc.parent_column_id = c.column_id
                                    WHERE fkc.constraint_object_id = fk.object_id
                                    ORDER BY fkc.constraint_column_id
                                    FOR XML PATH('')
                                ), 1, 2, ''),
                                OBJECT_NAME(fk.referenced_object_id),
                                STUFF((
                                    SELECT ', ' + QUOTENAME(c.name)
                                    FROM sys.foreign_key_columns fkc
                                    JOIN sys.columns c ON fkc.referenced_object_id = c.object_id AND fkc.referenced_column_id = c.column_id
                                    WHERE fkc.constraint_object_id = fk.object_id
                                    ORDER BY fkc.constraint_column_id
                                    FOR XML PATH('')
                                ), 1, 2, ''),
                                fk.update_referential_action_desc,
                                fk.delete_referential_action_desc
                            FROM sys.foreign_keys fk
                            WHERE fk.parent_object_id = OBJECT_ID(QUOTENAME(@schemaName) + '.' + QUOTENAME(@tableName));
    
                            OPEN fk_cursor;
                            FETCH NEXT FROM fk_cursor INTO @fk_name, @parent_columns, @referenced_table, 
                                                           @referenced_columns, @update_rule, @delete_rule;
    
                            WHILE @@FETCH_STATUS = 0
                            BEGIN
                                SET @sql = 'ALTER TABLE ' + QUOTENAME(@schemaName) + '.' + QUOTENAME(@new_tableName) 
                                         + ' ADD CONSTRAINT ' + QUOTENAME(@fk_name + '_FK') 
                                         + ' FOREIGN KEY (' + @parent_columns + ')' 
                                         + ' REFERENCES ' + QUOTENAME(@schemaName) + '.' + QUOTENAME(@referenced_table) 
                                         + ' (' + @referenced_columns + ')' 
                                         + ' ON UPDATE ' + @update_rule 
                                         + ' ON DELETE ' + @delete_rule;
        
                                BEGIN TRY
                                    EXEC sp_executesql @sql;
                                    PRINT '  外键约束创建完成: ' + @fk_name;
                                END TRY
                                BEGIN CATCH
                                    PRINT '  外键约束创建失败（可能引用表不存在）: ' + @fk_name;
                                END CATCH
        
                                FETCH NEXT FROM fk_cursor INTO @fk_name, @parent_columns, @referenced_table, 
                                                               @referenced_columns, @update_rule, @delete_rule;
                            END
    
                            CLOSE fk_cursor;
                            DEALLOCATE fk_cursor;
    
                            PRINT '表 ' + @new_tableName + ' 创建完成，包含所有索引和约束';
                        END
                        ELSE
                        BEGIN
                            PRINT '表 ' + @new_tableName + ' 已存在';
                        END
                
                '''
                cursor.execute(str_sql)
                connection.commit()
                bRes = True
        except Exception as e:
            output(f"[sqlserverMgr]复制表{tableName}时失败,错误:{e}",outputMode.saveLog,enum_Error.localInfo)
        return bRes
    def executeSql(connection,sql,bCommit = True,bGetRes = True):
        results = []
        try:
            with connection.cursor(as_dict=True) as cursor:
                if cursor is not None:
                    # 执行查询
                    cursor.execute(sql)
                    # 获取所有结果
                    if bGetRes:
                        results = cursor.fetchall()
                    if bCommit:
                        connection.commit()
        except Exception as e:
            output(f"[sqlserverMgr]执行查询时失败,错误:{e}\n{sql}",outputMode.saveLog,enum_Error.localInfo)
        return results

    def singleQuery(db_host,db_port,db_user,db_password,db_db,sql,bCommit = True,bGetRes = True):
        results = None
        try:
            with sqlserverMgr.getConnect(db_host,db_port,db_user,db_password,db_db) as connection:
                return sqlserverMgr.executeSql(connection,sql,bCommit,bGetRes)
        except Exception as e:
            pass
        return results

    def dualThread(self):
        db_host = self.db_host
        db_port = self.db_port
        db_user= self.db_user
        db_password = self.db_password
        db_name = self.db_name
        """处理接收"""
        output(f"[sqlserverMgr]:写sqlServer {db_host,db_port,db_name} 启动",outputMode.saveLog,enum_Error.localInfo)
        conn = None
        while self.is_run:
            try:
                # 连接数据库
                conn = sqlserverMgr.getConnect(db_host,db_port,db_user,db_password,db_name)
                if conn is None:
                    time.sleep(1)
                    continue
                #conn.autocommit = False
                nWriteTimeout = self.write_Event_timeout_ms / 1000 #转换成秒

                nTotalWriteCount = 1000
                nCurWriteCount = 0
                while self.is_run:
                    nCurWriteCount = nCurWriteCount+1
                    if nCurWriteCount> nTotalWriteCount:
                        break
                    self.writeEvent.wait(nWriteTimeout)
                    self.writeEvent.clear()
                    
                    # 执行普通批量更新慢
                    if self.databaseMgr is None:
                        continue
                    writeList = self.databaseMgr.copyWriteData()
                    while True:
                        index =None
                        if not writeList:
                            break
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
                            if bCommit is True:
                                conn.commit()
                            break
                        except Exception as e:
                            # 发生异常时回滚
                            conn.rollback()
                            tools.output(f"[sqlserverMgr]:执行SQL批量写入时发生错误: {e}",outputMode.saveLog,enum_Error.localInfo)
                            if sqlserverMgr.check_connection(conn) == False:
                                with self.writeLock:
                                    for tmp in writeList:
                                        self.databaseMgr.writeSql(tmp)
                                break
                            
                            if index is not None:
                                try:
                                    del writeList[index-1]#删除错误语句，从新执行插入
                                except Exception as e:
                                    output(f"[sqlserverMgr]:执行SQL批量写入时发生错误: {e},{writeList[index-1]}",outputMode.saveLog,enum_Error.localInfo)
                            else:
                                break
                conn = sqlserverMgr.closeConnect(conn)
                #conn = None
            except Exception as e:
                output(f"[sqlserverMgr]:{self._name}:连接数据库时发生错误: {e}",outputMode.saveLog,enum_Error.localInfo)
        conn = sqlserverMgr.closeConnect(conn)
        #try:
        #    if conn is not None:
        #        conn.close()
        #except Exception as e:
        #    output(f"[sqlserverMgr]:关闭数据库连接时发生错误: {e}",outputMode.saveLog,enum_Error.localInfo)
        output(f"[sqlserverMgr]:写sqlServer {db_host,db_port,db_name} 停止",outputMode.saveLog,enum_Error.localInfo)

