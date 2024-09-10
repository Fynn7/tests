from fastapi import FastAPI, HTTPException, Request
import pymysql
import json
from fastapi.encoders import jsonable_encoder
app = FastAPI()

db_config = {
    'host': 'localhost',
    'user': 'root',
    'password': 'root',
    'database': 'test_db'
}

@app.get("/alldata")
async def msg(request: Request):
    connection = pymysql.connect(**db_config)
    cursor = connection.cursor()

    sql = "SELECT name FROM employee"
    cursor.execute(sql)
    result = cursor.fetchall()

    items = []
    for row in result:
        item = {'name': row[0]}
        items.append(item)
    json_ques = jsonable_encoder(items)
    return json_ques

@app.post("/msg")
async def msg(request: Request):
    try:
        raw_data = await request.body()
        data = json.loads(raw_data)
        name = data.get('name')
        if not name:
            raise HTTPException(
                status_code=400, detail="Name parameter is required")

        connection = pymysql.connect(**db_config)
        cursor = connection.cursor()

        sql = "INSERT INTO employee (name) VALUES (%s);"
        cursor.execute(sql, (name,))
        connection.commit()
    except pymysql.MySQLError as e:
        raise HTTPException(status_code=500, detail=str(e))
    except json.JSONDecodeError:
        raise HTTPException(status_code=400, detail="Invalid JSON format")
    finally:
        cursor.close()
        connection.close()

    return {"message": "Name inserted successfully"}

@app.get("/data2")
def get_data2():
    return 'hk'

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=2136)
