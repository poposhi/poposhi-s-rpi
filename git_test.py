# -*- coding: utf-8 -*-

import pymysql.cursors
import datetime
import numpy as np

# Connect to the database
connection = pymysql.connect(host='localhost',
                             user='root',
                             password='qwerqwer',
                             db='test',
                             charset='utf8mb4',
                             cursorclass=pymysql.cursors.DictCursor)
#給資料庫，與編碼即可連上去
try:    #再給表格名 與資料名稱
    with connection.cursor() as cursor:
        # Create a new record  write sql_labguage 
        #sql = "INSERT INTO `motor` (`v`, `a`) VALUES (%s, %s)"
        #cursor.execute(sql, (1,2 ))
        
        time =datetime.datetime.now().isoformat()  #時間
        cursor.execute('INSERT INTO `motor` (`datetimeee`) VALUES(%s)', (time,))
       
        arr= a= np.array_str(np.arange(300))
        cursor.execute('INSERT INTO `motor` (`longtextttt`,`datetimeee`) VALUES(%s,%s)',
                       (arr,time))
        
    # connection is not autocommit by default. So you must commit to save
    # your changes.
    connection.commit()

    '''with connection.cursor() as cursor:
        # 我要選取id passward 從表格users 的email 
        sql = "SELECT `id`, `password` FROM `users` WHERE `email`=%s"
        cursor.execute(sql, ('webmaster@python.org',))
        result = cursor.fetchone()
        print(result)'''
finally:
    connection.close()