from ..logging_setup import logger

class DataChannelHandler:
    """데이터 채널 관리"""
    
    def __init__(self):
        self.data_channel = None
        self.on_message_callback = None
        self.on_open_callback = None
        self.on_close_callback = None
    
    def setup_channel(self, channel):
        """데이터 채널 핸들러 설정"""
        self.data_channel = channel
        
        @channel.on("open")
        def on_open():
            logger.info(f"데이터 채널 열림: {channel.label}")
            if self.on_open_callback:
                self.on_open_callback()
        
        @channel.on("close")
        def on_close():
            logger.info(f"데이터 채널 닫힘: {channel.label}")
            if self.on_close_callback:
                self.on_close_callback()

        @channel.on("message")
        def on_message(message):
            logger.info(f"메시지 수신: {message}")
            if self.on_message_callback:
                self.on_message_callback(message)
    
    def set_callbacks(self, on_message=None, on_open=None, on_close=None):
        """콜백 함수 설정"""
        self.on_message_callback = on_message
        self.on_open_callback = on_open
        self.on_close_callback = on_close
    
    def send_message(self, message):
        """메시지 전송"""
        if self.data_channel and self.data_channel.readyState == "open":
            self.data_channel.send(message)
            return True
        return False