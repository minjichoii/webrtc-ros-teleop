#!/usr/bin/env python3

from flask import Flask, render_template, request, session
from flask_socketio import SocketIO, emit, join_room
import platform

app = Flask(__name__)
app.config['SECRET_KEY'] = "wubba lubba dub dub"

socketio = SocketIO(app)

users_in_room = {}
rooms_sid = {}
names_sid = {}


@app.route("/join", methods=["GET"])
def join():
    display_name = request.args.get('display_name', 'Guest')  # 기본값 'Guest'
    mute_audio = request.args.get('mute_audio', '0')  # 기본값 '0'
    mute_video = request.args.get('mute_video', '0')  # 기본값 '0'
    room_id = request.args.get('room_id', '0000')  # 기본값 '0000'

    session[room_id] = {
        "name": display_name,
        "mute_audio": mute_audio,
        "mute_video": mute_video
    }
    
    return render_template("join.html",
                           room_id=room_id,
                           display_name=session[room_id]["name"],
                           mute_audio=session[room_id]["mute_audio"],
                           mute_video=session[room_id]["mute_video"])


@socketio.on("connect")
def on_connect():
    sid = request.sid
    print("New socket connected ", sid)


@socketio.on("join-room")
def on_join_room(data):
    sid = request.sid
    room_id = data["room_id"]
    display_name = session.get(room_id, {}).get("name", "Unknown")

    # register sid to the room
    join_room(room_id)
    rooms_sid[sid] = room_id
    names_sid[sid] = display_name

    # broadcast to others in the room
    print("[{}] New member joined: {}<{}>".format(room_id, display_name, sid))
    emit("user-connect", {"sid": sid, "name": display_name},
         broadcast=True, include_self=False, room=room_id)

    # add to user list maintained on server
    if room_id not in users_in_room:
        users_in_room[room_id] = [sid]
        emit("user-list", {"my_id": sid})  # send own id only
    else:
        usrlist = {u_id: names_sid[u_id]
                   for u_id in users_in_room[room_id]}
        # send list of existing users to the new member
        emit("user-list", {"list": usrlist, "my_id": sid})
        # add new member to user list maintained on server
        users_in_room[room_id].append(sid)

    print("\nusers: ", users_in_room, "\n")
    
@socketio.on("join")
def on_join(data):
    sid = request.sid
    room_id = data.get("room", "0000")  # 기본값 '0000'
    display_name = session.get(room_id, {}).get("name", "Unknown")

    # register sid to the room
    join_room(room_id)
    rooms_sid[sid] = room_id
    names_sid[sid] = display_name

    print(f"🚀 Client {sid} joined room: {room_id} as {display_name}")
    emit("room-joined", {"room": room_id}, room=room_id)

    # add to user list maintained on server
    if room_id not in users_in_room:
        users_in_room[room_id] = [sid]
        emit("user-list", {"my_id": sid})  # send own id only
    else:
        usrlist = {u_id: names_sid[u_id] for u_id in users_in_room[room_id]}
        emit("user-list", {"list": usrlist, "my_id": sid})  # send existing users
        users_in_room[room_id].append(sid)

    print("\nusers: ", users_in_room, "\n")
    
@socketio.on("message")
def handle_message(data):
    print(f"📩 Received message: {data}")  # 서버에서 메시지를 감지한 로그 출력
    emit("message", f"Echo: {data}", broadcast=True)  # 메시지를 다시 클라이언트에 전송


@socketio.on("disconnect")
def on_disconnect():
    sid = request.sid
    room_id = rooms_sid.get(sid, "Unknown")
    display_name = names_sid.get(sid, "Unknown")

    print("[{}] Member left: {}<{}>".format(room_id, display_name, sid))
    emit("user-disconnect", {"sid": sid},
         broadcast=True, include_self=False, room=room_id)

    if room_id in users_in_room and sid in users_in_room[room_id]:
        users_in_room[room_id].remove(sid)
        if len(users_in_room[room_id]) == 0:
            users_in_room.pop(room_id)

    rooms_sid.pop(sid, None)
    names_sid.pop(sid, None)

    print("\nusers: ", users_in_room, "\n")


@socketio.on("data")
def on_data(data):
    sender_sid = data['sender_id']
    target_sid = data['target_id']
    if sender_sid != request.sid:
        print("[Not supposed to happen!] request.sid and sender_id don't match!!!")

    if data["type"] != "new-ice-candidate":
        print('{} message from {} to {}'.format(
            data["type"], sender_sid, target_sid))
    socketio.emit('data', data, room=target_sid)


if __name__ == "__main__":
    import rospy
    rospy.init_node("flask_signaling_server", anonymous=True)
    
    print("🔥 Flask WebRTC Signaling Server Running on http://localhost:5000")
    socketio.run(app, host="0.0.0.0", port=5000, debug=True)

