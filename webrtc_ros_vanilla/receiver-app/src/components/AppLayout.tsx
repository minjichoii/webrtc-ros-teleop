import React, {ReactNode} from "react";

interface AppLayoutProps {
    children: ReactNode;
    title?: string;
}

const AppLayout: React.FC<AppLayoutProps> = ({
    children,
    title = 'TARO Remote Control'
}) => {
    return (
        <div 
            style={{
                display: 'flex',
                flexDirection: 'column',
                alignItems: 'center',
                justifyContent: 'center',
                height: '100vh',
                backgroundColor: '#f5f5f5',
                padding: '20px'
            }}
            >
            <h1 style={{
                textAlign: "center",
                color: "#333",
                margin: "0 0 20px 0",
                fontWeight: "bold"
            }}>{title}</h1>

            {children}
        </div>
    );
};

export default AppLayout;