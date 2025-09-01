mkdir cert && cd cert
openssl genrsa -out server_root_key.pem 2048
openssl req -x509 -new -nodes -key server_root_key.pem -sha256 -days 3650 -out server_root_cert.pem -subj "/CN=MyCA"


cat << EOF > server_csr.conf
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
EOF

openssl genrsa -out server.key 2048
openssl req -new -key server.key -out server.csr -config server_csr.cnf
openssl x509 -req -in server.csr -CA server_root_cert.pem -CAkey server_root_key.pem -CAcreateserial \
  -out server.crt -days 365 -sha256 -extensions req_ext -extfile server_csr.cnf

cd ..