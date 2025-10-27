from StT import StT

myStT = StT()
try:
    myStT.loop()
except Exception as e:
    myStT.close()
    print(f"Error: {e}")