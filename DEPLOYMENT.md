# Deployment Guide - Bilingual Translation System

This guide provides step-by-step instructions for deploying the bilingual (English/Urdu) documentation system to production.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Environment Configuration](#environment-configuration)
3. [Backend Deployment](#backend-deployment)
4. [Frontend Deployment](#frontend-deployment)
5. [Domain & SSL Setup](#domain--ssl-setup)
6. [Production Checklist](#production-checklist)
7. [Monitoring & Maintenance](#monitoring--maintenance)
8. [Troubleshooting](#troubleshooting)

---

## Prerequisites

### Required Tools

- **Node.js** 18+ and npm
- **Python** 3.8+
- **Docker** (optional, recommended)
- **Git** for version control
- **Domain name** with DNS access
- **SSL certificate** (Let's Encrypt recommended)

### Required Accounts

- **GitHub** account (for CI/CD)
- **Vercel** or **Netlify** account (for frontend hosting)
- **Heroku**, **Render**, or **AWS** account (for backend hosting)
- **Domain registrar** account (Cloudflare, Namecheap, etc.)

---

## Environment Configuration

### Backend Environment Variables

Create `backend/.env.production`:

```bash
# Server Configuration
HOST=0.0.0.0
PORT=8000
ENVIRONMENT=production
DEBUG=False

# CORS Settings
ALLOWED_ORIGINS=https://yourdomain.com,https://www.yourdomain.com

# Translation Model
MODEL_NAME=facebook/mbart-large-50-many-to-many-mmt
MODEL_CACHE_DIR=/app/models
DEVICE=cpu  # or 'cuda' if GPU available

# Cache Configuration
CACHE_TTL_SECONDS=86400  # 24 hours
CACHE_MAX_SIZE_BYTES=1073741824  # 1GB

# Rate Limiting
RATE_LIMIT_REQUESTS=100
RATE_LIMIT_WINDOW=60  # seconds

# Logging
LOG_LEVEL=INFO
LOG_FILE=/var/log/translation-api.log

# Monitoring (optional)
SENTRY_DSN=https://your-sentry-dsn@sentry.io/project-id

# API Keys (for RAG chatbot)
OPENAI_API_KEY=your_openai_api_key
COHERE_API_KEY=your_cohere_api_key
```

### Frontend Environment Variables

Create `.env.production`:

```bash
# API Configuration
REACT_APP_API_URL=https://api.yourdomain.com

# Analytics (optional)
REACT_APP_GA_MEASUREMENT_ID=G-XXXXXXXXXX

# Sentry (optional)
REACT_APP_SENTRY_DSN=https://your-sentry-dsn@sentry.io/project-id

# Environment
NODE_ENV=production
```

**Security Note:** Never commit `.env` files to Git. Use secret management services:
- GitHub Secrets for CI/CD
- AWS Secrets Manager
- Heroku Config Vars
- Vercel Environment Variables

---

## Backend Deployment

### Option 1: Docker Deployment (Recommended)

#### Step 1: Create Dockerfile

Create `backend/Dockerfile`:

```dockerfile
# Use official Python runtime
FROM python:3.10-slim

# Set working directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    curl \
    git \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements
COPY requirements.txt .
COPY pyproject.toml .

# Install Python dependencies
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY . .

# Download translation model (optional, can be done at runtime)
# RUN python -c "from transformers import MBartForConditionalGeneration, MBart50TokenizerFast; MBartForConditionalGeneration.from_pretrained('facebook/mbart-large-50-many-to-many-mmt'); MBart50TokenizerFast.from_pretrained('facebook/mbart-large-50-many-to-many-mmt')"

# Create non-root user
RUN useradd -m -u 1000 appuser && chown -R appuser:appuser /app
USER appuser

# Expose port
EXPOSE 8000

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=40s --retries=3 \
    CMD curl -f http://localhost:8000/api/translate/health || exit 1

# Start application
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000", "--workers", "2"]
```

#### Step 2: Create docker-compose.yml

```yaml
version: '3.8'

services:
  backend:
    build:
      context: ./backend
      dockerfile: Dockerfile
    container_name: translation-backend
    ports:
      - "8000:8000"
    environment:
      - HOST=0.0.0.0
      - PORT=8000
      - ENVIRONMENT=production
    env_file:
      - backend/.env.production
    volumes:
      - model-cache:/app/models
      - logs:/var/log
    restart: unless-stopped
    networks:
      - translation-network

  nginx:
    image: nginx:alpine
    container_name: translation-nginx
    ports:
      - "80:80"
      - "443:443"
    volumes:
      - ./nginx/nginx.conf:/etc/nginx/nginx.conf
      - ./nginx/ssl:/etc/nginx/ssl
    depends_on:
      - backend
    restart: unless-stopped
    networks:
      - translation-network

volumes:
  model-cache:
  logs:

networks:
  translation-network:
    driver: bridge
```

#### Step 3: Build and Run

```bash
# Build Docker image
docker-compose build

# Start services
docker-compose up -d

# Check logs
docker-compose logs -f backend

# Verify health
curl http://localhost:8000/api/translate/health
```

---

### Option 2: Heroku Deployment

#### Step 1: Create Heroku App

```bash
# Install Heroku CLI
curl https://cli-assets.heroku.com/install.sh | sh

# Login
heroku login

# Create app
heroku create your-translation-api

# Set environment variables
heroku config:set ENVIRONMENT=production
heroku config:set ALLOWED_ORIGINS=https://yourdomain.com
heroku config:set MODEL_NAME=facebook/mbart-large-50-many-to-many-mmt

# Set Python buildpack
heroku buildpacks:set heroku/python
```

#### Step 2: Create Procfile

Create `backend/Procfile`:

```
web: uvicorn main:app --host 0.0.0.0 --port $PORT --workers 2
```

#### Step 3: Deploy

```bash
# Initialize git (if not already)
cd backend
git init
heroku git:remote -a your-translation-api

# Deploy
git add .
git commit -m "Initial deployment"
git push heroku main

# Check logs
heroku logs --tail

# Verify deployment
curl https://your-translation-api.herokuapp.com/api/translate/health
```

---

### Option 3: AWS EC2 Deployment

#### Step 1: Launch EC2 Instance

1. **Login to AWS Console** → EC2
2. **Launch Instance:**
   - AMI: Ubuntu Server 22.04 LTS
   - Instance Type: t3.medium (minimum)
   - Storage: 30GB EBS
   - Security Group: Allow ports 22, 80, 443, 8000

#### Step 2: Connect and Setup

```bash
# SSH into instance
ssh -i your-key.pem ubuntu@your-ec2-ip

# Update system
sudo apt update && sudo apt upgrade -y

# Install Python and dependencies
sudo apt install -y python3.10 python3-pip python3-venv git nginx

# Clone repository
git clone https://github.com/your-org/roboticAI_book.git
cd roboticAI_book/backend

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install --upgrade pip
pip install -r requirements.txt

# Create systemd service
sudo nano /etc/systemd/system/translation-api.service
```

**Service File Content:**

```ini
[Unit]
Description=Translation API Service
After=network.target

[Service]
Type=simple
User=ubuntu
WorkingDirectory=/home/ubuntu/roboticAI_book/backend
Environment="PATH=/home/ubuntu/roboticAI_book/backend/venv/bin"
ExecStart=/home/ubuntu/roboticAI_book/backend/venv/bin/uvicorn main:app --host 0.0.0.0 --port 8000 --workers 2
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

```bash
# Enable and start service
sudo systemctl daemon-reload
sudo systemctl enable translation-api
sudo systemctl start translation-api

# Check status
sudo systemctl status translation-api
```

---

## Frontend Deployment

### Option 1: Vercel Deployment (Recommended for Docusaurus)

#### Step 1: Install Vercel CLI

```bash
npm install -g vercel
```

#### Step 2: Login and Deploy

```bash
# Login to Vercel
vercel login

# Deploy
vercel

# Follow prompts:
# ? Set up and deploy "roboticAI_book"? [Y/n] Y
# ? Which scope? your-team
# ? Link to existing project? [y/N] N
# ? What's your project's name? roboticai-book
# ? In which directory is your code located? ./

# Deploy to production
vercel --prod
```

#### Step 3: Configure Environment Variables

```bash
# Set environment variables
vercel env add REACT_APP_API_URL production
# Enter: https://api.yourdomain.com

# Or via Vercel Dashboard:
# 1. Go to project settings
# 2. Environment Variables
# 3. Add REACT_APP_API_URL
```

#### Step 4: Configure Domain

```bash
# Add custom domain
vercel domains add yourdomain.com
vercel domains add www.yourdomain.com

# Follow DNS setup instructions
```

---

### Option 2: Netlify Deployment

#### Step 1: Create netlify.toml

```toml
[build]
  command = "npm run build"
  publish = "build"

[build.environment]
  NODE_VERSION = "18"

[[redirects]]
  from = "/api/*"
  to = "https://api.yourdomain.com/:splat"
  status = 200
  force = true

[[redirects]]
  from = "/*"
  to = "/index.html"
  status = 200
```

#### Step 2: Deploy via CLI

```bash
# Install Netlify CLI
npm install -g netlify-cli

# Login
netlify login

# Initialize site
netlify init

# Deploy
netlify deploy --prod

# Set environment variables
netlify env:set REACT_APP_API_URL https://api.yourdomain.com
```

---

### Option 3: GitHub Pages Deployment

**Note:** GitHub Pages is static-only and requires backend to be hosted separately.

#### Step 1: Update docusaurus.config.js

```javascript
module.exports = {
  title: 'Physical AI & Humanoid Robotics',
  url: 'https://your-username.github.io',
  baseUrl: '/roboticAI_book/',
  organizationName: 'your-username',
  projectName: 'roboticAI_book',
  deploymentBranch: 'gh-pages',

  // ... rest of config
};
```

#### Step 2: Deploy

```bash
# Build and deploy
GIT_USER=your-username npm run deploy

# Or with SSH
USE_SSH=true npm run deploy
```

---

## Domain & SSL Setup

### Option 1: Cloudflare (Recommended)

#### Step 1: Add Site to Cloudflare

1. Login to Cloudflare
2. Add your domain
3. Update nameservers at your registrar

#### Step 2: Configure DNS Records

```
Type  Name              Content                      Proxy  TTL
A     yourdomain.com    your-backend-ip              Yes    Auto
CNAME www               yourdomain.com               Yes    Auto
CNAME api               your-backend-domain.com      Yes    Auto
```

#### Step 3: Enable SSL/TLS

1. Go to **SSL/TLS** → **Overview**
2. Set mode to **Full (strict)**
3. Go to **Edge Certificates**
4. Enable **Always Use HTTPS**
5. Enable **Automatic HTTPS Rewrites**

---

### Option 2: Let's Encrypt with Certbot

```bash
# Install Certbot
sudo apt install certbot python3-certbot-nginx

# Obtain certificate
sudo certbot --nginx -d yourdomain.com -d www.yourdomain.com -d api.yourdomain.com

# Auto-renewal
sudo certbot renew --dry-run

# Add cron job for auto-renewal
echo "0 12 * * * /usr/bin/certbot renew --quiet" | sudo tee -a /etc/crontab > /dev/null
```

---

## Production Checklist

### Pre-Deployment

- [ ] All environment variables set
- [ ] Secrets not committed to Git
- [ ] Backend health endpoint returns 200
- [ ] Frontend builds without errors
- [ ] CORS origins configured correctly
- [ ] Rate limiting enabled
- [ ] Logging configured
- [ ] Error tracking (Sentry) set up

### Security

- [ ] SSL/TLS enabled (HTTPS)
- [ ] CORS properly configured
- [ ] API rate limiting active
- [ ] Sensitive data encrypted
- [ ] Regular security updates scheduled
- [ ] Firewall rules configured
- [ ] DDoS protection enabled (via Cloudflare)

### Performance

- [ ] CDN enabled for static assets
- [ ] Gzip/Brotli compression enabled
- [ ] Browser caching configured
- [ ] Translation model preloaded
- [ ] Database indexes created (if using DB)
- [ ] Load balancing configured (if needed)

### Monitoring

- [ ] Uptime monitoring (UptimeRobot, Pingdom)
- [ ] Error tracking (Sentry)
- [ ] Analytics (Google Analytics)
- [ ] Performance monitoring (Web Vitals)
- [ ] Log aggregation (CloudWatch, Loggly)
- [ ] Alert notifications configured

### Documentation

- [ ] Deployment runbook created
- [ ] Incident response plan documented
- [ ] Contact information updated
- [ ] Backup and recovery procedures documented

---

## Monitoring & Maintenance

### Health Checks

**Backend Health Endpoint:**
```bash
curl https://api.yourdomain.com/api/translate/health

# Expected response:
{
  "status": "healthy",
  "model_loaded": true,
  "uptime_seconds": 3600,
  "cache_entries": 150
}
```

**Frontend Health Check:**
```bash
curl -I https://yourdomain.com

# Expected:
HTTP/2 200
content-type: text/html
```

### Monitoring Tools

#### 1. UptimeRobot (Free)

- Monitor: `https://yourdomain.com`
- Monitor: `https://api.yourdomain.com/api/translate/health`
- Check interval: 5 minutes
- Alert via: Email, SMS, Slack

#### 2. Sentry Error Tracking

**Backend Integration:**
```python
# backend/main.py
import sentry_sdk
from sentry_sdk.integrations.fastapi import FastApiIntegration

sentry_sdk.init(
    dsn=os.getenv("SENTRY_DSN"),
    integrations=[FastApiIntegration()],
    environment="production",
    traces_sample_rate=0.1,
)
```

**Frontend Integration:**
```javascript
// src/index.js
import * as Sentry from "@sentry/react";

Sentry.init({
  dsn: process.env.REACT_APP_SENTRY_DSN,
  environment: "production",
  tracesSampleRate: 0.1,
});
```

#### 3. Google Analytics

```javascript
// src/theme/Root.js
import ReactGA from 'react-ga4';

if (process.env.REACT_APP_GA_MEASUREMENT_ID) {
  ReactGA.initialize(process.env.REACT_APP_GA_MEASUREMENT_ID);
}
```

### Regular Maintenance Tasks

**Daily:**
- Check error rates in Sentry
- Review uptime status

**Weekly:**
- Review application logs
- Check cache hit rates
- Monitor API response times

**Monthly:**
- Review and rotate logs
- Update dependencies
- Security patch updates
- Performance audit with Lighthouse

**Quarterly:**
- Comprehensive security audit
- Load testing
- Disaster recovery drill
- Review and update documentation

---

## Troubleshooting

### Issue: Backend Not Starting

**Check logs:**
```bash
# Docker
docker-compose logs backend

# Systemd
sudo journalctl -u translation-api -f

# Heroku
heroku logs --tail
```

**Common causes:**
- Missing environment variables
- Port already in use
- Model download failed
- Insufficient memory

**Fix:**
```bash
# Check environment variables
env | grep ENVIRONMENT

# Check port availability
sudo lsof -i :8000

# Restart service
sudo systemctl restart translation-api
```

---

### Issue: CORS Errors

**Symptom:** Console shows "CORS policy blocked"

**Fix:**
```python
# backend/main.py
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://yourdomain.com",
        "https://www.yourdomain.com"
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

---

### Issue: SSL Certificate Expired

**Check expiration:**
```bash
echo | openssl s_client -servername yourdomain.com -connect yourdomain.com:443 2>/dev/null | openssl x509 -noout -dates
```

**Renew with Certbot:**
```bash
sudo certbot renew
sudo systemctl reload nginx
```

---

### Issue: High Memory Usage

**Monitor:**
```bash
# Check memory usage
free -h
docker stats

# Check process memory
ps aux --sort=-%mem | head -n 10
```

**Fix:**
```bash
# Reduce number of workers
# In Dockerfile or Procfile:
CMD ["uvicorn", "main:app", "--workers", "1"]

# Upgrade instance size
# Or implement cache eviction
```

---

## Rollback Procedure

### Vercel Rollback

```bash
# List deployments
vercel ls

# Rollback to previous deployment
vercel rollback [deployment-url]
```

### Heroku Rollback

```bash
# List releases
heroku releases

# Rollback to previous release
heroku rollback v[version-number]
```

### Docker Rollback

```bash
# List images
docker images

# Run previous version
docker-compose down
docker run -d -p 8000:8000 your-image:previous-tag
```

---

## Cost Estimation

### Small Traffic (< 1000 users/month)

| Service | Provider | Cost |
|---------|----------|------|
| Frontend | Vercel (Hobby) | Free |
| Backend | Heroku (Basic) | $7/month |
| Domain | Namecheap | $10/year |
| SSL | Let's Encrypt | Free |
| **Total** | | **~$9/month** |

### Medium Traffic (< 10,000 users/month)

| Service | Provider | Cost |
|---------|----------|------|
| Frontend | Vercel (Pro) | $20/month |
| Backend | AWS EC2 (t3.medium) | $30/month |
| Domain | Cloudflare (Pro) | $20/month |
| Storage | AWS S3 | $5/month |
| **Total** | | **~$75/month** |

### High Traffic (< 100,000 users/month)

| Service | Provider | Cost |
|---------|----------|------|
| Frontend | Vercel (Pro) | $20/month |
| Backend | AWS ECS (2x t3.large) | $120/month |
| Load Balancer | AWS ALB | $16/month |
| Domain | Cloudflare (Business) | $200/month |
| Monitoring | Sentry (Team) | $26/month |
| **Total** | | **~$382/month** |

---

## Support & Resources

### Documentation
- [FastAPI Deployment](https://fastapi.tiangolo.com/deployment/)
- [Docusaurus Deployment](https://docusaurus.io/docs/deployment)
- [Vercel Documentation](https://vercel.com/docs)
- [Heroku Python Support](https://devcenter.heroku.com/articles/python-support)

### Community
- GitHub Issues: `https://github.com/your-org/roboticAI_book/issues`
- Discord/Slack: [Your community link]

### Emergency Contacts
- System Admin: admin@yourdomain.com
- On-call Engineer: +1-XXX-XXX-XXXX

---

## Summary

This deployment guide covered:
- ✅ Environment configuration
- ✅ Backend deployment (Docker, Heroku, AWS)
- ✅ Frontend deployment (Vercel, Netlify, GitHub Pages)
- ✅ Domain and SSL setup
- ✅ Production checklist
- ✅ Monitoring and maintenance
- ✅ Troubleshooting procedures
- ✅ Rollback procedures
- ✅ Cost estimation

**Recommended Stack for Production:**
- Frontend: Vercel
- Backend: AWS EC2 or Heroku
- DNS/CDN: Cloudflare
- Monitoring: Sentry + UptimeRobot
- Analytics: Google Analytics 4

**Estimated Setup Time:** 4-6 hours for first deployment

For questions or issues, refer to the troubleshooting section or open a GitHub issue.
