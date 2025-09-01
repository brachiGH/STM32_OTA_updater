## Setup Instructions

Create a virtual environment:
   ```
   python -m venv venv
   ```

Activate the virtual environment:
   - On Windows:
     ```
     venv\Scripts\activate
     ```
   - On macOS/Linux:
     ```
     source venv/bin/activate
     ```
Install the required dependencies:
   ```
   pip install -r requirements.txt
   ```

## Usage

To run the API server, execute the following command:

1. Create a CA Key and Certificate
```
mkdir cert && cd cert

# Generate CA private key (2048-bit RSA)
openssl genrsa -out server_root_key.pem 2048

# Create self-signed CA X.509 certificate
openssl req -x509 -new -nodes -key server_root_key.pem -sha256 -days 3650 -out server_root_cert.pem \
  -subj "/CN=MyCA"
```
2. Create a Server Key and Certificate Signing Request (CSR)

Create a config file server_csr.cnf (adjust CN / SAN entries):

```
[ req ]
default_bits       = 2048
prompt             = no
default_md         = sha256
distinguished_name = dn
req_extensions     = req_ext

[ dn ]
CN = controlserver.local

[ req_ext ]
subjectAltName = @alt_names

[ alt_names ]
IP.1 = 127.0.0.1
DNS.1 = localhost
DNS.2 = controlserver.local
DNS.3 = www.controlserver.local
```

Generate the server key and CSR:

```
openssl genrsa -out server.key 2048
openssl req -new -key server.key -out server.csr -config server_csr.cnf
```

3. Sign the Server Certificate with the CA

```
openssl x509 -req -in server.csr -CA server_root_cert.pem -CAkey server_root_key.pem -CAcreateserial \
  -out server.crt -days 365 -sha256 -extensions req_ext -extfile server_csr.cnf
```

You can add the CA cert to your Linux trust store:
```
sudo cp server_root_cert.pem /usr/local/share/ca-certificates/
sudo update-ca-certificates
```

Files to Use
Server:
  Private key: server.key
  Certificate: server.crt
Client:
  CA certificate: server_root_cert.pem

```
cd ..
python server.py 
//or
uvicorn server:app --host 0.0.0.0 --port 8443 --ssl-keyfile=cert/server.key --ssl-certfile=cert/server.crt --reload


//not recommended
sudo -E ./venv/bin/python server.py 
//or
sudo -E ./venv/bin/uvicorn server:app --host 0.0.0.0 --port 443 --ssl-keyfile=cert/server.key --ssl-certfile=cert/server.crt --reload
```
